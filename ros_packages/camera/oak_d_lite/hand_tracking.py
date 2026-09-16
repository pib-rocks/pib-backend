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
