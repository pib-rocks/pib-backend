"""Post-processing for the three-blob hand tracking pipeline.

Decoder format evidence:
https://github.com/geaxgx/depthai_hand_tracker/blob/main/custom_models/generate_postproc_onnx.py
defines the output as ``[top_k, 8]`` and the runtime manager reads each record
as ``score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y``.  The vendored
blob metadata reports ``result: [8, 10]`` (DepthAI dimension order), hence 80
flattened values.  The head consumes detector tensors ``classificators``
(1x896x1 anchors) and ``regressors`` (1x896x18: bbox plus seven palm
keypoints), and emits anchor-decoded normalized coordinates after NMS.

The landmark network exposes its crop-space image landmarks in
``Identity_dense/BiasAdd/Add`` as 21 XYZ triples, with confidence in
``Identity_1``.  ``Identity_3_dense/BiasAdd/Add`` is the *metric world*
landmark head of the same MediaPipe graph, and conversions of this model
frequently omit it entirely, so it is only a fallback.  The runtime node maps
the landmarks back through the ``NNData.getTransformation()`` attached by
DepthAI, and falls back to the palm ROI when that transform is absent. Both
``(1, 1)`` scores and ``(1, 63)`` landmark vectors are flattened before
assembly into a ``Detection``.
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
LANDMARK_SCORE_THRESHOLD = 0.5
LANDMARK_VALUE_COUNT = LANDMARK_COUNT * 3
LANDMARK_SCORE_LAYER = "Identity_1"
# The crop-space landmark head comes first: MediaPipe's ``Identity`` output is
# renamed ``Identity_dense/BiasAdd/Add`` by the OpenVINO conversion.  The
# ``Identity_3`` variant is the metric world-landmark head, which is centred on
# the hand instead of the crop and is missing from several published blobs, so
# it is tried only after the image-space heads.
LANDMARK_XYZ_LAYERS = (
    "Identity_dense/BiasAdd/Add",
    "Identity",
    "Identity_3_dense/BiasAdd/Add",
)
# Crop-space XY in this blob is normalised to 0..1. Values above this are
# treated as already being in landmark-input pixels (MediaPipe's 0..224).
LANDMARK_NORMALIZED_PEAK = 1.5
# Sub-pixel margin that keeps a derived crop strictly inside its source frame.
MANIP_CROP_INSET_PIXELS = 0.5


@dataclass(frozen=True)
class ManipCrop:
    """Normalized full-source crop plus its network target size."""

    center_x: float
    center_y: float
    width: float
    height: float
    output_width: int
    output_height: int


def fit_manip_crop(
    source_width: int,
    source_height: int,
    output_width: int,
    output_height: int,
    inset: float = MANIP_CROP_INSET_PIXELS,
) -> ManipCrop:
    """Derive a valid ImageManip crop and target size from measured input dimensions.

    ImageManip validates its crop against the frame it actually receives and
    rejects the whole frame with ``Initial crop is outside the source image``
    when the rect does not fit.  A rect assumed at build time is therefore
    unusable as soon as the camera delivers other dimensions.  The crop here
    uses the full measured source minus a sub-pixel inset. Warp-cache limits are
    enforced by bounding the camera branch itself, not by narrowing this crop or
    chaining manipulations. The target size is always the network input size.
    """
    if source_width <= 0 or source_height <= 0:
        raise ValueError("manip source must have positive dimensions")
    if output_width <= 0 or output_height <= 0:
        raise ValueError("manip output must have positive dimensions")
    inset_x = max(0.0, min(float(inset), source_width / 4.0))
    inset_y = max(0.0, min(float(inset), source_height / 4.0))
    crop_width = source_width - 2.0 * inset_x
    crop_height = source_height - 2.0 * inset_y
    return ManipCrop(
        center_x=0.5,
        center_y=0.5,
        width=crop_width / source_width,
        height=crop_height / source_height,
        output_width=int(output_width),
        output_height=int(output_height),
    )


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
        """Return the decoded ROI normalized to the ImageManip source crop.

        The palm ImageManip warps its full rectangular crop to the square
        network input with ``setOutputSize``. It does not letterbox that crop,
        so the decoder's normalized axes are already the source-frame axes.
        """
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError("palm ROI frame must have positive dimensions")
        return (
            self.roi_x,
            self.roi_y,
            self.roi_size,
            self.roi_size,
        )


def _square_to_frame(
    x: float,
    y: float,
    frame_width: int,
    frame_height: int,
    source_width: int = None,
    source_height: int = None,
) -> Tuple[float, float]:
    # The palm input is a direct warp of the complete rectangular source crop,
    # not a letterboxed square. Normalized decoder coordinates therefore map
    # independently onto the declared output width and height. The source
    # dimensions remain accepted because callers also use them for the
    # landmark packet transformation path.
    del source_width, source_height
    return (
        min(float(frame_width), max(0.0, x * frame_width)),
        min(float(frame_height), max(0.0, y * frame_height)),
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


def landmark_score(tensor: Iterable[float]) -> float:
    """Read the landmark presence score, including a batched ``(1, 1)`` tensor."""
    values = np.asarray(tensor, dtype=np.float32).reshape(-1)
    if values.size != 1:
        raise ValueError("landmark confidence must contain one value")
    score = float(values[0])
    if not math.isfinite(score):
        raise ValueError("landmark confidence is not finite")
    return score


def landmark_xyz(tensor: Sequence[float]) -> np.ndarray:
    """Return 21 XYZ triples from a landmark tensor, including ``(1, 63)``."""
    values = np.asarray(tensor, dtype=np.float32)
    if values.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    if values.size != LANDMARK_VALUE_COUNT:
        raise ValueError("landmark result must contain exactly 63 values (21 x 3)")
    values = values.reshape(LANDMARK_COUNT, 3)
    if not np.all(np.isfinite(values)):
        raise ValueError("landmark result contains non-finite values")
    return values


def landmarks_in_crop_pixels(
    tensor: Sequence[float], landmark_input_size: int = 224
) -> np.ndarray:
    """Return crop-space XYZ with XY in landmark-input pixels."""
    values = landmark_xyz(tensor)
    if values.size == 0:
        return values
    peak = float(np.max(np.abs(values[:, :2])))
    if peak <= LANDMARK_NORMALIZED_PEAK:
        values = np.array(values, copy=True)
        values[:, :2] *= float(landmark_input_size)
    return values


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
    values = landmarks_in_crop_pixels(tensor, landmark_input_size)
    if values.size == 0:
        return []

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
