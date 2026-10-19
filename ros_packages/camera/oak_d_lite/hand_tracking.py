"""Post-processing for the three-blob hand tracking pipeline.

Decoder format evidence:
https://github.com/geaxgx/depthai_hand_tracker/blob/main/custom_models/generate_postproc_onnx.py
defines the output as ``[top_k, 8]`` and the runtime manager reads each record
as ``score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y``.  The chain
is fed by the zoo's own decoding head (``palm_detection_128x128_decoding``),
which consumes the detector's NNData (``classificators`` anchors, ``regressors``
bbox plus seven palm keypoints), runs NMS on the edge and returns the TOP 10
most confident records.  The head's ``result`` layer is a Concat of 1 + 3 + 2 + 2
fields, read straight from the zoo IR, so the eight fields per record are
``score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y`` as above.

The head only yields usable numbers when its input datatype matches the
detector's: the zoo detector outputs FP16, so the head has to be compiled
without ``-ip U8``.  A candidate compiled with the image-input default emits a
score column stuck at a constant 1.0 and coordinates above 1.0 that correlate
with nothing in the frame - that is what produced the phantom hands.

The landmark network exposes its crop-space image landmarks in
``Identity_dense/BiasAdd/Add`` as 21 XYZ triples, with a presence score in
``Identity_1``.  The third component is the model's hand-relative depth, which
is why ``relative_landmark_z`` publishes it next to the pixels of x and y.
The score is usable and gated on, exactly as the reference does: measured with a
good crop it reads 0.998, while an unusable crop reads 0.003 to 0.018.  An
earlier note here claimed the head arrives unactivated and could never be
compared against a threshold; that came from another build's documentation.
``Identity_3_dense/BiasAdd/Add`` is the *metric world*
landmark head of the same MediaPipe graph, and conversions of this model
frequently omit it entirely, so it is only a fallback.  The runtime node maps
the landmarks back through the ``NNData.getTransformation()`` attached by
DepthAI, and falls back to the palm ROI when that transform is absent. Both
``(1, 1)`` scores and ``(1, 63)`` landmark vectors are flattened before
assembly into a ``Detection``.
"""

from dataclasses import dataclass
import marshal
import math
from string import Template
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
# Ten records: the zoo decoding head runs NMS on the edge and returns the TOP 10
# most confident candidates.  The score gate below prunes the background ones
# before they cost a landmark crop, so the count is the blob's layout, not a cap.
# ``pd_postprocessing_top2_sh1`` is the other layout this decoder accepts: the
# same eight fields, TOP 2, which is what the device-side fast script reads.
PALM_RESULT_COUNT = 10
TOP2_PALM_RESULT_COUNT = 2
PALM_RESULT_WIDTH = 8
PALM_SCORE_THRESHOLD = 0.5
LANDMARK_COUNT = 21
LANDMARK_VALUE_COUNT = LANDMARK_COUNT * 3
LANDMARK_SCORE_LAYER = "Identity_1"
# The same blob reports the hand's handedness in ``Identity_2``.  The reference
# turns it into the label ("right" if handedness > 0.5 else "left"); here it is
# published as a scalar and the label stays "hand", which is what the model
# store and the imitation chain already emit.
LANDMARK_HANDEDNESS_LAYER = "Identity_2"
# The reference gates every landmark result with ``if lm_score > 0.5`` before it
# is used, so a crop that produced no hand is dropped instead of published.
LANDMARK_SCORE_THRESHOLD = 0.5
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
    tensor: Iterable[float],
    score_threshold: float = PALM_SCORE_THRESHOLD,
    max_hands: int = 10,
) -> List[PalmRegion]:
    """Parse the post-processing layer, rejecting any unexpected layout.

    ``max_hands`` bounds the candidates the head is allowed to contribute; the
    zoo head itself returns ten and ``pd_postprocessing_top2_sh1`` returns two.
    Both layouts are eight fields per record.  ``score_threshold`` does the
    real filtering: only records that pass it are turned into a landmark crop,
    so a frame that contains one hand costs one crop, not ten.
    """
    values = np.asarray(tensor, dtype=np.float32).reshape(-1)
    accepted = (
        PALM_RESULT_COUNT * PALM_RESULT_WIDTH,
        TOP2_PALM_RESULT_COUNT * PALM_RESULT_WIDTH,
    )
    if values.size not in accepted:
        raise ValueError(
            "palm decoder result must contain exactly "
            f"{PALM_RESULT_COUNT * PALM_RESULT_WIDTH} values "
            f"({PALM_RESULT_COUNT} detections x {PALM_RESULT_WIDTH}) "
            "or exactly "
            f"{TOP2_PALM_RESULT_COUNT * PALM_RESULT_WIDTH} values "
            f"({TOP2_PALM_RESULT_COUNT} detections x {PALM_RESULT_WIDTH})"
        )
    values = values.reshape(values.size // PALM_RESULT_WIDTH, PALM_RESULT_WIDTH)
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
    palms.sort(key=lambda palm: palm.score, reverse=True)
    if max_hands > 0:
        palms = palms[:max_hands]
    return palms


def landmark_score(tensor: Iterable[float]) -> float:
    """Read the reported landmark presence score, including a ``(1, 1)`` tensor.

    Callers compare this against ``LANDMARK_SCORE_THRESHOLD`` as the reference
    does; an unusable crop scores an order of magnitude below it.
    """
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


def relative_landmark_z(
    tensor: Sequence[float], landmark_input_size: int = 224
) -> List[float]:
    """Return the 21 hand-relative z values, in the scale of x and y.

    The reference (`pib-rocks/imitation`, ``template_manager_script_duo.py``)
    keeps all three landmark components in one unitless scale by dividing the
    crop-space values by the landmark input size::

        rrn_lms[3*i+2] /= lm_input_size

    Its finger angles are then computed from those three-component vectors, so
    the relative depth really drives the result.  This function reproduces that
    normalisation and, unlike ``landmarks_in_crop_pixels``, leaves the z alone
    rather than turning it into pixels.

    The z is signed and small.  Anything that clamps the landmark components
    into 0..1 (``depthai_nodes``' ``KeypointParser`` does exactly that) destroys
    it, which is why the hand-written chain is the path that can carry it.
    """
    values = landmark_xyz(tensor)
    if values.size == 0:
        return []
    peak = float(np.max(np.abs(values[:, :2])))
    if peak <= LANDMARK_NORMALIZED_PEAK:
        scale = 1.0
    else:
        scale = 1.0 / float(landmark_input_size)
    return [float(value) * scale for value in values[:, 2]]


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
    return map_crop_points_to_frame(
        values,
        palm,
        frame_width,
        frame_height,
        landmark_input_size,
        source_width,
        source_height,
    )


def map_crop_points_to_frame(
    values: np.ndarray,
    palm: PalmRegion,
    frame_width: int,
    frame_height: int,
    landmark_input_size: int,
    source_width: int = None,
    source_height: int = None,
) -> List[Tuple[float, float]]:
    """Map crop-pixel XYZ points of any count through the shared ROI geometry."""
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


# Device-side hand tracker. The crop configuration is computed by the Script
# below and linked into ImageManip on the device. The host only receives the
# marshalled detection result.
FAST_MODEL_ID = "hand_tracking_fast"
# Device lpb.NNData returns these FP16 layers. Host getTensor is not on the device.
FAST_FP16_LAYERS = (
    ("result", 16),
    ("Identity_1", 1),
    ("Identity_2", 1),
    ("Identity_dense/BiasAdd/Add", 63),
)
# Smaller camera output than the 2104x1560 ISP frame. One manipulation from
# that ISP frame down to the 128 palm input fails with
# WARP_SWCH_ERR_CACHE_TOO_SMALL. 256x144 keeps the published 16:9 aspect, and
# its long side is exactly twice the palm input, the ratio the hand chain
# already uses.
FAST_BRANCH_WIDTH = 256
FAST_BRANCH_HEIGHT = 144
FAST_PALM_INPUT_SIZE = 128
FAST_LANDMARK_INPUT_SIZE = 224
FAST_SINGLE_HAND_TOLERANCE = 10
FAST_RESULT_OUTPUT = "detections"
FAST_PALM_CONFIG_OUTPUT = "pre_pd_manip_cfg"
FAST_LANDMARK_CONFIG_OUTPUT = "pre_lm_manip_cfg"
FAST_DECODER_INPUT = "from_post_pd_nn"
FAST_LANDMARK_INPUT = "from_lm_nn"
_FAST_RESULT_KEYS = (
    "lm_score",
    "handedness",
    "palm_score",
    "rotation",
    "rect_center_x",
    "rect_center_y",
    "rect_size",
    "rrn_lms",
)

_FAST_TRACKER_SCRIPT = r"""
import marshal
from math import sin, cos, atan2, pi, degrees, floor, dist

pad_h = ${pad_h}
pad_w = ${pad_w}
img_h = ${img_h}
img_w = ${img_w}
frame_size = ${frame_size}
palm_input = ${palm_input}
lm_input_size = ${landmark_input}
pd_score_thresh = ${pd_score_thresh}
lm_score_thresh = ${lm_score_thresh}
single_hand_tolerance = ${single_hand_tolerance}

class BufferMgr:
    def __init__(self):
        self._bufs = {}
    def __call__(self, size):
        try:
            buf = self._bufs[size]
        except KeyError:
            buf = self._bufs[size] = Buffer(size)
        return buf

buffer_mgr = BufferMgr()

def send_result(result):
    result_serial = marshal.dumps(result)
    buffer = buffer_mgr(len(result_serial))
    buffer.getData()[:] = result_serial
    node.io['${result_output}'].send(buffer)

def send_hands(lm_score, handedness, palm_score, rotation, rect_center_x, rect_center_y, rect_size, rrn_lms):
    send_result(dict([
        ("lm_score", lm_score),
        ("handedness", handedness),
        ("palm_score", palm_score),
        ("rotation", rotation),
        ("rect_center_x", rect_center_x),
        ("rect_center_y", rect_center_y),
        ("rect_size", rect_size),
        ("rrn_lms", rrn_lms),
    ]))

def normalize_radians(angle):
    return angle - 2 * pi * floor((angle + pi) / (2 * pi))

def tensor_values(nn_data, name):
    # lpb.NNData on the device returns a float list from getLayerFp16.
    # getTensor is the host binding and is absent on the device.
    tensor = nn_data.getLayerFp16(name)
    values = []
    pending = [tensor]
    while pending:
        item = pending.pop()
        if isinstance(item, (int, float)):
            values.append(float(item))
            continue
        try:
            count = len(item)
        except Exception:
            values.append(float(item))
            continue
        if isinstance(item, str):
            values.append(float(item))
            continue
        for index in range(count - 1, -1, -1):
            pending.append(item[index])
    return values

cfg_pre_pd = ImageManipConfig()
cfg_pre_pd.setOutputSize(palm_input, palm_input, ImageManipConfig.ResizeMode.LETTERBOX)

id_index_mcp = 5
id_middle_mcp = 9
id_ring_mcp = 13
ids_for_bounding_box = [0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18]

send_new_frame_to_branch = 1
detected_hands = []
single_hand_count = 0

while True:
    if send_new_frame_to_branch == 1:
        node.io['${palm_config_output}'].send(cfg_pre_pd)
        detection = tensor_values(node.io['${decoder_input}'].get(), "result")
        hands = []
        if len(detection) >= 16:
            for i in range(2):
                pd_score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y = detection[i*8:(i+1)*8]
                if pd_score >= pd_score_thresh and box_size > 0:
                    kp02_x = kp2_x - kp0_x
                    kp02_y = kp2_y - kp0_y
                    sqn_rr_size = 2.9 * box_size
                    rotation = 0.5 * pi - atan2(-kp02_y, kp02_x)
                    rotation = normalize_radians(rotation)
                    sqn_rr_center_x = box_x + 0.5 * box_size * sin(rotation)
                    sqn_rr_center_y = box_y - 0.5 * box_size * cos(rotation)
                    hands.append([sqn_rr_size, rotation, sqn_rr_center_x, sqn_rr_center_y, float(pd_score)])
        if len(hands) == 0:
            send_hands([], [], [], [], [], [], [], [])
            send_new_frame_to_branch = 1
            single_hand_count = 0
            continue
        detected_hands = hands

    hand_landmarks = dict([
        ("lm_score", []),
        ("handedness", []),
        ("palm_score", []),
        ("rotation", []),
        ("rect_center_x", []),
        ("rect_center_y", []),
        ("rect_size", []),
        ("rrn_lms", []),
    ])
    updated_detect_hands = []
    last_hand = len(detected_hands) - 1
    for i, hand in enumerate(detected_hands):
        sqn_rr_size, rotation, sqn_rr_center_x, sqn_rr_center_y, palm_score = hand
        rr = RotatedRect()
        rr.center.x = sqn_rr_center_x
        rr.center.y = (sqn_rr_center_y * frame_size - pad_h) / img_h
        rr.size.width = sqn_rr_size
        rr.size.height = sqn_rr_size * frame_size / img_h
        rr.angle = degrees(rotation)
        cfg = ImageManipConfig()
        cfg.addCropRotatedRect(rr, True)
        cfg.setOutputSize(lm_input_size, lm_input_size, ImageManipConfig.ResizeMode.STRETCH)
        reuse_prev_image = True if len(detected_hands) > 1 and i == last_hand else False
        cfg.setReusePreviousImage(reuse_prev_image)
        node.io['${landmark_config_output}'].send(cfg)

    for hand in detected_hands:
        sqn_rr_size, rotation, sqn_rr_center_x, sqn_rr_center_y, palm_score = hand
        lm_result = node.io['${landmark_input_port}'].get()
        lm_score = tensor_values(lm_result, "Identity_1")[0]
        if lm_score > lm_score_thresh:
            handedness = tensor_values(lm_result, "Identity_2")[0]
            rrn_raw = tensor_values(lm_result, "Identity_dense/BiasAdd/Add")
            rrn_lms = rrn_raw[:]
            cos_rot = cos(rotation)
            sin_rot = sin(rotation)
            sqn_lms = []
            for k in range(21):
                rrn_lms[3*k] = rrn_lms[3*k] / lm_input_size
                rrn_lms[3*k+1] = rrn_lms[3*k+1] / lm_input_size
                rrn_lms[3*k+2] = rrn_lms[3*k+2] / lm_input_size
                sqn_x = sqn_rr_center_x + sqn_rr_size * ((rrn_lms[3*k] - 0.5) * cos_rot + (0.5 - rrn_lms[3*k+1]) * sin_rot)
                sqn_y = sqn_rr_center_y + sqn_rr_size * ((rrn_lms[3*k+1] - 0.5) * cos_rot + (rrn_lms[3*k] - 0.5) * sin_rot)
                sqn_lms.append(sqn_x)
                sqn_lms.append(sqn_y)
            hand_landmarks["lm_score"].append(lm_score)
            hand_landmarks["handedness"].append(handedness)
            hand_landmarks["palm_score"].append(palm_score)
            hand_landmarks["rotation"].append(rotation)
            hand_landmarks["rect_center_x"].append(sqn_rr_center_x)
            hand_landmarks["rect_center_y"].append(sqn_rr_center_y)
            hand_landmarks["rect_size"].append(sqn_rr_size)
            hand_landmarks["rrn_lms"].append(rrn_raw)
            x0 = sqn_lms[0]
            y0 = sqn_lms[1]
            x1 = 0.25 * (sqn_lms[2*id_index_mcp] + sqn_lms[2*id_ring_mcp]) + 0.5 * sqn_lms[2*id_middle_mcp]
            y1 = 0.25 * (sqn_lms[2*id_index_mcp+1] + sqn_lms[2*id_ring_mcp+1]) + 0.5 * sqn_lms[2*id_middle_mcp+1]
            rotation = 0.5 * pi - atan2(y0 - y1, x1 - x0)
            rotation = normalize_radians(rotation)
            min_x = min_y = 1
            max_x = max_y = 0
            for landmark_id in ids_for_bounding_box:
                min_x = min(min_x, sqn_lms[2*landmark_id])
                max_x = max(max_x, sqn_lms[2*landmark_id])
                min_y = min(min_y, sqn_lms[2*landmark_id+1])
                max_y = max(max_y, sqn_lms[2*landmark_id+1])
            axis_aligned_center_x = 0.5 * (max_x + min_x)
            axis_aligned_center_y = 0.5 * (max_y + min_y)
            cos_rot = cos(rotation)
            sin_rot = sin(rotation)
            min_x = min_y = 1
            max_x = max_y = -1
            for landmark_id in ids_for_bounding_box:
                original_x = sqn_lms[2*landmark_id] - axis_aligned_center_x
                original_y = sqn_lms[2*landmark_id+1] - axis_aligned_center_y
                projected_x = original_x * cos_rot + original_y * sin_rot
                projected_y = -original_x * sin_rot + original_y * cos_rot
                min_x = min(min_x, projected_x)
                max_x = max(max_x, projected_x)
                min_y = min(min_y, projected_y)
                max_y = max(max_y, projected_y)
            projected_center_x = 0.5 * (max_x + min_x)
            projected_center_y = 0.5 * (max_y + min_y)
            center_x = (projected_center_x * cos_rot - projected_center_y * sin_rot + axis_aligned_center_x)
            center_y = (projected_center_x * sin_rot + projected_center_y * cos_rot + axis_aligned_center_y)
            width = (max_x - min_x)
            height = (max_y - min_y)
            sqn_rr_size = 2 * max(width, height)
            sqn_rr_center_x = (center_x + 0.1 * height * sin_rot)
            sqn_rr_center_y = (center_y - 0.1 * height * cos_rot)
            hand[0] = sqn_rr_size
            hand[1] = rotation
            hand[2] = sqn_rr_center_x
            hand[3] = sqn_rr_center_y
            updated_detect_hands.append(hand)
    detected_hands = updated_detect_hands

    if len(detected_hands) == 2:
        dist_rr_centers = dist([detected_hands[0][2], detected_hands[0][3]], [detected_hands[1][2], detected_hands[1][3]])
        if dist_rr_centers < 0.02:
            if hand_landmarks["lm_score"][0] > hand_landmarks["lm_score"][1]:
                pop_i = 1
            else:
                pop_i = 0
            for key in hand_landmarks:
                hand_landmarks[key].pop(pop_i)
            detected_hands.pop(pop_i)

    nb_hands = len(detected_hands)
    if nb_hands == 2 and (hand_landmarks["handedness"][0] - 0.5) * (hand_landmarks["handedness"][1] - 0.5) > 0:
        for key in hand_landmarks:
            hand_landmarks[key].pop(1)
        detected_hands.pop(1)
        nb_hands = 1

    if nb_hands == 1:
        single_hand_count = single_hand_count + 1
    else:
        single_hand_count = 0

    send_hands(hand_landmarks["lm_score"], hand_landmarks["handedness"], hand_landmarks["palm_score"], hand_landmarks["rotation"], hand_landmarks["rect_center_x"], hand_landmarks["rect_center_y"], hand_landmarks["rect_size"], hand_landmarks["rrn_lms"])

    if nb_hands == 0:
        send_new_frame_to_branch = 1
    elif nb_hands == 1 and single_hand_count >= single_hand_tolerance:
        send_new_frame_to_branch = 1
        single_hand_count = 0
    else:
        send_new_frame_to_branch = 2
"""


def letterbox_square(source_width: int, source_height: int) -> Tuple[int, int, int]:
    """Return the square side and the padding that letterboxes this frame.

    The palm network sees a square letterbox, so its coordinates live on that
    square. The landmark crop is taken from the unpadded camera frame, which
    is why the vertical pad has to be removed again before a point is scaled
    onto the published 16:9 frame.
    """
    if source_width <= 0 or source_height <= 0:
        raise ValueError("fast hand source must have positive dimensions")
    side = max(int(source_width), int(source_height))
    pad_w = (side - int(source_width)) // 2
    pad_h = (side - int(source_height)) // 2
    return side, pad_w, pad_h


def validate_fast_branch_size(
    source_width: int,
    source_height: int,
    palm_input: int = FAST_PALM_INPUT_SIZE,
) -> float:
    """Refuse a branch whose long side is more than twice the palm input.

    2104x1560 into 128x128 is the manipulation that fails with
    ``WARP_SWCH_ERR_CACHE_TOO_SMALL``. The accepted branch stays within 2:1.
    """
    if source_width <= 0 or source_height <= 0 or palm_input <= 0:
        raise ValueError("fast hand branch must have positive dimensions")
    scale = max(int(source_width), int(source_height)) / float(palm_input)
    if scale > 2.0:
        raise ValueError(
            f"Refusing {source_width}x{source_height} to {palm_input}: "
            "one image manipulation from the 2104x1560 ISP frame fails with "
            "WARP_SWCH_ERR_CACHE_TOO_SMALL; the long side must stay within 2:1"
        )
    return scale


def build_fast_tracker_script(
    source_width: int,
    source_height: int,
    palm_input: int = FAST_PALM_INPUT_SIZE,
    landmark_input: int = FAST_LANDMARK_INPUT_SIZE,
) -> str:
    """Return the device Script that computes both crop configurations."""
    validate_fast_branch_size(source_width, source_height, palm_input)
    side, pad_w, pad_h = letterbox_square(source_width, source_height)
    return Template(_FAST_TRACKER_SCRIPT).substitute(
        pad_h=pad_h,
        pad_w=pad_w,
        img_h=int(source_height),
        img_w=int(source_width),
        frame_size=side,
        palm_input=int(palm_input),
        landmark_input=int(landmark_input),
        pd_score_thresh=repr(float(PALM_SCORE_THRESHOLD)),
        lm_score_thresh=repr(float(LANDMARK_SCORE_THRESHOLD)),
        single_hand_tolerance=int(FAST_SINGLE_HAND_TOLERANCE),
        result_output=FAST_RESULT_OUTPUT,
        palm_config_output=FAST_PALM_CONFIG_OUTPUT,
        landmark_config_output=FAST_LANDMARK_CONFIG_OUTPUT,
        decoder_input=FAST_DECODER_INPUT,
        landmark_input_port=FAST_LANDMARK_INPUT,
    )


def build_fast_hand_graph(
    pipeline,
    source_output,
    palm,
    decoder,
    landmark,
    source_width: int,
    source_height: int,
):
    """Build the two-stage graph and return its one host queue.

    That queue is ``detections``. It carries the marshalled hand result.
    Frames and crop configurations stay on the device:

    - ``pre_pd_manip_cfg`` links into the palm ImageManip ``inputConfig``
    - ``pre_lm_manip_cfg`` links into the landmark ImageManip ``inputConfig``
    - ``from_post_pd_nn`` receives the decoder output
    - ``from_lm_nn`` receives the landmark output

    Each network uses the SHAVE count recorded for its blob. The composite
    budget is the sum of those three records.
    """
    validate_fast_branch_size(source_width, source_height, int(palm.input_width))
    if (int(palm.input_width), int(palm.input_height)) != (
        FAST_PALM_INPUT_SIZE,
        FAST_PALM_INPUT_SIZE,
    ):
        raise ValueError("fast palm network input must be 128x128")
    if (int(landmark.input_width), int(landmark.input_height)) != (
        FAST_LANDMARK_INPUT_SIZE,
        FAST_LANDMARK_INPUT_SIZE,
    ):
        raise ValueError("fast landmark network input must be 224x224")

    import depthai as dai

    script_node = pipeline.create(dai.node.Script)
    script_node.setScript(
        build_fast_tracker_script(
            source_width,
            source_height,
            palm_input=int(palm.input_width),
            landmark_input=int(landmark.input_width),
        )
    )

    palm_manip = pipeline.create(dai.node.ImageManip)
    palm_manip.setMaxOutputFrameSize(int(palm.input_width) * int(palm.input_height) * 3)
    palm_manip.inputConfig.setWaitForMessage(True)
    palm_manip.inputImage.setMaxSize(1)
    palm_manip.inputImage.setBlocking(False)
    source_output.link(palm_manip.inputImage)
    script_node.outputs[FAST_PALM_CONFIG_OUTPUT].link(palm_manip.inputConfig)

    palm_nn = pipeline.create(dai.node.NeuralNetwork)
    palm_nn.setBlobPath(palm.blob_path)
    palm_nn.setNumShavesPerInferenceThread(palm.shaves)
    palm_manip.out.link(palm_nn.input)

    decoder_nn = pipeline.create(dai.node.NeuralNetwork)
    decoder_nn.setBlobPath(decoder.blob_path)
    decoder_nn.setNumShavesPerInferenceThread(decoder.shaves)
    palm_nn.out.link(decoder_nn.input)
    decoder_nn.out.link(script_node.inputs[FAST_DECODER_INPUT])

    landmark_manip = pipeline.create(dai.node.ImageManip)
    landmark_manip.setMaxOutputFrameSize(
        int(landmark.input_width) * int(landmark.input_height) * 3
    )
    landmark_manip.inputConfig.setWaitForMessage(True)
    landmark_manip.inputImage.setMaxSize(1)
    landmark_manip.inputImage.setBlocking(False)
    source_output.link(landmark_manip.inputImage)
    script_node.outputs[FAST_LANDMARK_CONFIG_OUTPUT].link(landmark_manip.inputConfig)

    landmark_nn = pipeline.create(dai.node.NeuralNetwork)
    landmark_nn.setBlobPath(landmark.blob_path)
    landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
    landmark_manip.out.link(landmark_nn.input)
    landmark_nn.out.link(script_node.inputs[FAST_LANDMARK_INPUT])

    return script_node.outputs[FAST_RESULT_OUTPUT].createOutputQueue(
        maxSize=1, blocking=False
    )


@dataclass(frozen=True)
class FastHand:
    """One device result, still in the letterboxed square and the crop."""

    roi_x: float
    roi_y: float
    roi_size: float
    rotation: float
    crop_landmarks: Tuple[float, ...]
    landmark_score: float
    handedness: float
    palm_score: float


@dataclass(frozen=True)
class FastDetection:
    """Landmarks and box in published-frame pixels."""

    landmark_score: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    keypoint_x: Tuple[float, ...]
    keypoint_y: Tuple[float, ...]
    keypoint_z: Tuple[float, ...]
    handedness: float
    palm_score: float


def _payload_bytes(payload) -> bytes:
    if isinstance(payload, (bytes, bytearray)):
        return bytes(payload)
    getter = getattr(payload, "getData", None)
    data = payload if getter is None else getter()
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    tobytes = getattr(data, "tobytes", None)
    if callable(tobytes):
        return tobytes()
    return bytes(data)


def _finite_float(value, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"fast hand {label} is not finite")
    return number


def parse_fast_script_result(payload) -> List[FastHand]:
    """Read the device Script's detection result. No frames, no crop configs."""
    try:
        result = marshal.loads(_payload_bytes(payload))
    except (EOFError, TypeError, ValueError) as exc:
        raise ValueError(f"fast hand result is not a detection payload: {exc}") from exc
    if not isinstance(result, dict):
        raise ValueError("fast hand result must be a detection record")
    missing = [key for key in _FAST_RESULT_KEYS if key not in result]
    if missing:
        raise ValueError(f"fast hand result is missing {missing[0]}")
    scores = list(result["lm_score"])
    count = len(scores)
    columns = {}
    for key in _FAST_RESULT_KEYS:
        if key == "rrn_lms":
            continue
        values = list(result[key])
        if len(values) != count:
            raise ValueError(f"fast hand result {key} does not match lm_score")
        columns[key] = values
    landmarks = list(result["rrn_lms"])
    if len(landmarks) != count:
        raise ValueError("fast hand result rrn_lms does not match lm_score")

    hands = []
    for index, score in enumerate(scores):
        landmark_score = _finite_float(score, "landmark score")
        if landmark_score < LANDMARK_SCORE_THRESHOLD:
            continue
        crop = landmark_xyz(landmarks[index])
        hands.append(
            FastHand(
                roi_x=_finite_float(columns["rect_center_x"][index], "roi x"),
                roi_y=_finite_float(columns["rect_center_y"][index], "roi y"),
                roi_size=_finite_float(columns["rect_size"][index], "roi size"),
                rotation=_finite_float(columns["rotation"][index], "rotation"),
                crop_landmarks=tuple(float(value) for value in crop.reshape(-1)),
                landmark_score=landmark_score,
                handedness=_finite_float(columns["handedness"][index], "handedness"),
                palm_score=_finite_float(columns["palm_score"][index], "palm score"),
            )
        )
    return hands


def _published_from_square(
    x: float,
    y: float,
    source_width: int,
    source_height: int,
    frame_width: int,
    frame_height: int,
) -> Tuple[float, float]:
    side, pad_w, pad_h = letterbox_square(source_width, source_height)
    del side
    image_x = min(float(source_width), max(0.0, x - pad_w))
    image_y = min(float(source_height), max(0.0, y - pad_h))
    return (
        image_x * float(frame_width) / float(source_width),
        image_y * float(frame_height) / float(source_height),
    )


def _bbox_from_points(
    points: Sequence[Tuple[float, float]], frame_width: int, frame_height: int
) -> Tuple[int, int, int, int]:
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    x_min = max(0, min(frame_width - 1, int(math.floor(min(xs)))))
    y_min = max(0, min(frame_height - 1, int(math.floor(min(ys)))))
    x_max = max(x_min + 1, min(frame_width, int(math.ceil(max(xs)))))
    y_max = max(y_min + 1, min(frame_height, int(math.ceil(max(ys)))))
    return x_min, y_min, x_max, y_max


def map_fast_hand(
    hand: FastHand,
    frame_width: int,
    frame_height: int,
    source_width: int,
    source_height: int,
) -> FastDetection:
    """Map one device result onto the published frame.

    The Script's ROI is in the letterboxed square, which is the coordinate
    space ``map_landmarks_to_frame`` already uses. Removing the letterbox pad
    and scaling each axis puts the point on the 16:9 frame.
    """
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("published frame must have positive dimensions")
    if hand.roi_size <= 0:
        raise ValueError("fast hand ROI must have positive size")
    side, _pad_w, _pad_h = letterbox_square(source_width, source_height)
    palm = PalmRegion(
        score=hand.palm_score,
        box_x=hand.roi_x,
        box_y=hand.roi_y,
        box_size=hand.roi_size,
        roi_x=hand.roi_x,
        roi_y=hand.roi_y,
        roi_size=hand.roi_size,
        rotation=hand.rotation,
    )
    square_points = map_landmarks_to_frame(
        hand.crop_landmarks,
        palm,
        side,
        side,
        FAST_LANDMARK_INPUT_SIZE,
    )
    if len(square_points) != LANDMARK_COUNT:
        raise ValueError("fast hand result must contain 21 landmarks")
    points = [
        _published_from_square(
            x, y, source_width, source_height, frame_width, frame_height
        )
        for x, y in square_points
    ]
    x_min, y_min, x_max, y_max = _bbox_from_points(points, frame_width, frame_height)
    depth = relative_landmark_z(hand.crop_landmarks, FAST_LANDMARK_INPUT_SIZE)
    return FastDetection(
        landmark_score=hand.landmark_score,
        x_min=x_min,
        y_min=y_min,
        x_max=x_max,
        y_max=y_max,
        keypoint_x=tuple(point[0] for point in points),
        keypoint_y=tuple(point[1] for point in points),
        keypoint_z=tuple(depth),
        handedness=hand.handedness,
        palm_score=hand.palm_score,
    )
