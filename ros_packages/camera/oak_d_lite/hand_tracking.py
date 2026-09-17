"""Post-processing for the three-blob hand tracking pipeline.

Decoder format evidence:
https://github.com/geaxgx/depthai_hand_tracker/blob/main/custom_models/generate_postproc_onnx.py
defines the output as ``[top_k, 8]`` and the runtime manager reads each record
as ``score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y``.  The vendored
blob metadata reports ``result: [8, 10]`` (DepthAI dimension order), hence 80
flattened values.  The head consumes detector tensors ``classificators``
(1x896x1 anchors) and ``regressors`` (1x896x18: bbox plus seven palm
keypoints), and emits anchor-decoded normalized coordinates after NMS.

The landmark network exposes normalized image landmarks in
``Identity_3_dense/BiasAdd/Add`` as 21 XYZ triples, with confidence in
``Identity_1``. The runtime node maps them back through the
``NNData.getTransformation()`` attached by DepthAI.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

HAND_KEYPOINT_NAMES = (
    "wrist",
    "thumb_cmc",
    "thumb_mcp",
    "thumb_ip",
    "thumb_tip",
    "index_finger_mcp",
    "index_finger_pip",
    "index_finger_dip",
    "index_finger_tip",
    "middle_finger_mcp",
    "middle_finger_pip",
    "middle_finger_dip",
    "middle_finger_tip",
    "ring_finger_mcp",
    "ring_finger_pip",
    "ring_finger_dip",
    "ring_finger_tip",
    "pinky_mcp",
    "pinky_pip",
    "pinky_dip",
    "pinky_tip",
)
PALM_RESULT_COUNT = 10
PALM_RESULT_WIDTH = 8
LANDMARK_COUNT = 21
# Sub-pixel margin that keeps a derived crop strictly inside its source frame.
MANIP_CROP_INSET_PIXELS = 0.5
# Largest downscale a single ImageManip may be asked for.  RVC2 resolves a
# manipulation through its warp cache, and that cache has to hold every source
# pixel one output block reads: the steeper the ratio, the more source lines per
# block.  Beyond the budget the device reports ``WARP_SWCH_ERR_CACHE_TO_SMALL``
# and skips the frame, and the accompanying ``Initial crop is outside the source
# image`` is a symptom of the same rejection rather than a crop that truly hangs
# over the edge.  Halving stays well inside the budget, so a manipulation that
# needs more is split into successive halvings.
MANIP_MAX_DOWNSCALE_PER_STAGE = 2.0


@dataclass(frozen=True)
class ManipCrop:
    """Normalized crop plus target size for one ImageManip stage."""

    center_x: float
    center_y: float
    width: float
    height: float
    output_width: int
    output_height: int


def manip_downscale_factor(
    source_width: float,
    source_height: float,
    output_width: int,
    output_height: int,
) -> float:
    """Return the downscale ratio one manipulation would have to resolve."""
    if output_width <= 0 or output_height <= 0:
        raise ValueError("manip output must have positive dimensions")
    return max(
        float(source_width) / float(output_width),
        float(source_height) / float(output_height),
    )


def fit_manip_crop(
    source_width: int,
    source_height: int,
    output_width: int,
    output_height: int,
    inset: float = MANIP_CROP_INSET_PIXELS,
    max_downscale: float = MANIP_MAX_DOWNSCALE_PER_STAGE,
) -> ManipCrop:
    """Derive a valid ImageManip crop and target size from measured input dimensions.

    ImageManip validates its crop against the frame it actually receives and
    rejects the whole frame with ``Initial crop is outside the source image``
    when the rect does not fit.  A rect assumed at build time is therefore
    unusable as soon as the camera delivers other dimensions.  The crop here
    starts at the full source minus a sub-pixel inset and is narrowed to what
    the warp cache can resolve in one manipulation, so the rect fits any source
    and stays inside the budget.  The target size is always the network input
    size: a frame trimmed to the source instead would not match the tensor the
    model expects.
    """
    if source_width <= 0 or source_height <= 0:
        raise ValueError("manip source must have positive dimensions")
    if output_width <= 0 or output_height <= 0:
        raise ValueError("manip output must have positive dimensions")
    if max_downscale <= 0:
        raise ValueError("manip downscale budget must be positive")
    inset_x = max(0.0, min(float(inset), source_width / 4.0))
    inset_y = max(0.0, min(float(inset), source_height / 4.0))
    crop_width = min(source_width - 2.0 * inset_x, max_downscale * output_width)
    crop_height = min(source_height - 2.0 * inset_y, max_downscale * output_height)
    return ManipCrop(
        center_x=0.5,
        center_y=0.5,
        width=crop_width / source_width,
        height=crop_height / source_height,
        output_width=int(output_width),
        output_height=int(output_height),
    )


def plan_manip_stages(
    source_width: int,
    source_height: int,
    output_width: int,
    output_height: int,
    max_downscale: float = MANIP_MAX_DOWNSCALE_PER_STAGE,
) -> List[Tuple[int, int]]:
    """Split one manipulation into stages that each stay inside the warp cache.

    Taking a 640x480 branch to the 128x128 palm input in one step asks the warp
    engine for five source pixels per output pixel, which is what exhausts the
    cache.  Each intermediate stage halves both dimensions, so the source aspect
    ratio is preserved exactly and the letterbox padding the palm decoder
    normalizes against is introduced only by the final stage.
    """
    if source_width <= 0 or source_height <= 0:
        raise ValueError("manip source must have positive dimensions")
    if output_width <= 0 or output_height <= 0:
        raise ValueError("manip output must have positive dimensions")
    if max_downscale <= 1.0:
        raise ValueError("manip downscale budget must exceed 1.0")

    stages: List[Tuple[int, int]] = []
    width, height = int(source_width), int(source_height)
    while (
        manip_downscale_factor(width, height, output_width, output_height)
        > max_downscale
    ):
        next_width, next_height = width // 2, height // 2
        if next_width <= 0 or next_height <= 0:
            break
        # Halving past the target only adds a stage that upscales again.
        next_factor = manip_downscale_factor(
            next_width, next_height, output_width, output_height
        )
        if next_factor < 1.0:
            break
        width, height = next_width, next_height
        stages.append((width, height))
    stages.append((int(output_width), int(output_height)))
    return stages


@dataclass(frozen=True)
class PalmRegion:
    score: float
    box_x: float
    box_y: float
    box_size: float
    roi_x: float
    roi_y: float
    roi_size: float
    rotation: float

    def bbox_pixels(
        self,
        frame_width: int,
        frame_height: int,
        source_width: int = None,
        source_height: int = None,
    ) -> Tuple[int, ...]:
        half = self.box_size / 2.0
        x_min, y_min = _square_to_frame(
            self.box_x - half,
            self.box_y - half,
            frame_width,
            frame_height,
            source_width,
            source_height,
        )
        x_max, y_max = _square_to_frame(
            self.box_x + half,
            self.box_y + half,
            frame_width,
            frame_height,
            source_width,
            source_height,
        )
        return (
            int(round(x_min)),
            int(round(y_min)),
            int(round(x_max)),
            int(round(y_max)),
        )

    def roi_for_frame(
        self, frame_width: int, frame_height: int
    ) -> Tuple[float, float, float, float]:
        """Return center and size normalized to the unpadded source frame."""
        frame_size = float(max(frame_width, frame_height))
        pad_x = (frame_size - frame_width) / 2.0
        pad_y = (frame_size - frame_height) / 2.0
        return (
            (self.roi_x * frame_size - pad_x) / frame_width,
            (self.roi_y * frame_size - pad_y) / frame_height,
            self.roi_size * frame_size / frame_width,
            self.roi_size * frame_size / frame_height,
        )


def _square_to_frame(
    x: float,
    y: float,
    frame_width: int,
    frame_height: int,
    source_width: int = None,
    source_height: int = None,
) -> Tuple[float, float]:
    source_width = source_width or frame_width
    source_height = source_height or frame_height
    frame_size = float(max(source_width, source_height))
    pad_x = (frame_size - source_width) / 2.0
    pad_y = (frame_size - source_height) / 2.0
    normalized_x = (x * frame_size - pad_x) / source_width
    normalized_y = (y * frame_size - pad_y) / source_height
    return (
        min(float(frame_width), max(0.0, normalized_x * frame_width)),
        min(float(frame_height), max(0.0, normalized_y * frame_height)),
    )


def decode_palm_result(
    tensor: Iterable[float], score_threshold: float = 0.5
) -> List[PalmRegion]:
    """Parse the top-10 decoder layer, rejecting any unexpected layout."""
    values = np.asarray(tensor, dtype=np.float32)
    if values.size != PALM_RESULT_COUNT * PALM_RESULT_WIDTH:
        raise ValueError(
            "palm decoder result must contain exactly 80 values " "(10 detections x 8)"
        )
    values = values.reshape(PALM_RESULT_COUNT, PALM_RESULT_WIDTH)
    palms = []
    for score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y in values:
        if not np.all(
            np.isfinite((score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y))
        ):
            continue
        if score < score_threshold or box_size <= 0:
            continue
        delta_x = kp2_x - kp0_x
        delta_y = kp2_y - kp0_y
        rotation = 0.5 * math.pi - math.atan2(-delta_y, delta_x)
        rotation -= 2 * math.pi * math.floor((rotation + math.pi) / (2 * math.pi))
        palms.append(
            PalmRegion(
                score=float(score),
                box_x=float(box_x),
                box_y=float(box_y),
                box_size=float(box_size),
                roi_x=float(box_x + 0.5 * box_size * math.sin(rotation)),
                roi_y=float(box_y - 0.5 * box_size * math.cos(rotation)),
                roi_size=float(2.9 * box_size),
                rotation=float(rotation),
            )
        )
    return palms


def map_landmarks_to_frame(
    tensor: Sequence[float],
    palm: PalmRegion,
    frame_width: int,
    frame_height: int,
    landmark_input_size: int = 224,
    source_width: int = None,
    source_height: int = None,
) -> List[Tuple[float, float]]:
    """Map 21 crop-space XYZ triples to full-frame pixel XY coordinates."""
    values = np.asarray(tensor, dtype=np.float32)
    if values.size == 0:
        return []
    if values.size != LANDMARK_COUNT * 3:
        raise ValueError("landmark result must contain exactly 63 values (21 x 3)")
    values = values.reshape(LANDMARK_COUNT, 3)
    if not np.all(np.isfinite(values)):
        raise ValueError("landmark result contains non-finite values")

    normalized = values[:, :2] / float(landmark_input_size)
    cos_rotation = math.cos(palm.rotation)
    sin_rotation = math.sin(palm.rotation)
    points = []
    for crop_x, crop_y in normalized:
        image_x = palm.roi_x + palm.roi_size * (
            (crop_x - 0.5) * cos_rotation + (0.5 - crop_y) * sin_rotation
        )
        image_y = palm.roi_y + palm.roi_size * (
            (crop_y - 0.5) * cos_rotation + (crop_x - 0.5) * sin_rotation
        )
        points.append(
            _square_to_frame(
                image_x,
                image_y,
                frame_width,
                frame_height,
                source_width,
                source_height,
            )
        )
    return points
