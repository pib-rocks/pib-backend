"""Host-side translation helpers for face-crop classifier chains."""

import math

import numpy as np

EMOTION_LABELS = ("neutral", "happy", "sad", "surprise", "anger")
EMOTION_OUTPUT_LAYER = "prob_emotion"
FACEMESH_INPUT_SIZE = 192
FACEMESH_LANDMARK_COUNT = 468
FACEMESH_OUTPUT_LAYER = "conv2d_210"
FACE_DETECTOR_MODEL_ID = "face_detection_yunet_160x120"
FACE_CROP_PADDING = 0.1
HEAD_POSE_OUTPUTS = (
    ("yaw", "angle_y_fc"),
    ("pitch", "angle_p_fc"),
    ("roll", "angle_r_fc"),
)


def packet_timestamp(packet):
    """Normalise a DepthAI packet timestamp to a sortable key."""
    stamp = packet.getTimestamp()
    seconds = getattr(stamp, "sec", None)
    if seconds is None:
        seconds = getattr(stamp, "seconds", 0)
    nanos = getattr(stamp, "nanosec", None)
    if nanos is None:
        nanos = getattr(stamp, "microseconds", 0) * 1000
    return int(seconds), int(nanos)


def softmax(values):
    """Return stable softmax probabilities for one classifier output."""
    logits = np.asarray(values, dtype=np.float64).reshape(-1)
    if logits.size != len(EMOTION_LABELS):
        raise ValueError(
            f"emotion output has {logits.size} values, expected {len(EMOTION_LABELS)}"
        )
    if not np.all(np.isfinite(logits)):
        raise ValueError("emotion output contains a non-finite value")
    shifted = logits - np.max(logits)
    exponentials = np.exp(shifted)
    total = float(np.sum(exponentials))
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("emotion output cannot be normalised")
    return exponentials / total


def _pixel(value, extent):
    return min(extent, max(0, int(float(value) * extent)))


def _face_box_pixels(face, frame_width, frame_height):
    box = face.getBoundingBox()
    half_width = float(box.size.width) / 2.0
    half_height = float(box.size.height) / 2.0
    return (
        _pixel(float(box.center.x) - half_width, frame_width),
        _pixel(float(box.center.y) - half_height, frame_height),
        _pixel(float(box.center.x) + half_width, frame_width),
        _pixel(float(box.center.y) + half_height, frame_height),
    )


def emotion_logits(packet, preferred_layer=EMOTION_OUTPUT_LAYER):
    """Read the classifier's sole output without assuming its exported name."""
    names_getter = getattr(packet, "getAllLayerNames", None)
    names = []
    if callable(names_getter):
        names = [str(name) for name in names_getter()]
    if preferred_layer in names or not names:
        try:
            return packet.getTensor(preferred_layer)
        except (KeyError, RuntimeError):
            if not names:
                raise
    if len(names) == 1:
        return packet.getTensor(names[0])
    raise ValueError(
        "emotion classifier must expose one output layer; found "
        + ", ".join(sorted(names))
    )


def emotion_probabilities(packet, preferred_layer=EMOTION_OUTPUT_LAYER):
    """Return the classifier's probabilities without assuming they are logits.

    The shipped blob ends in a SoftMax layer - the OpenVINO XML's last operations are
    Convolution, Const, Convert, Add, SoftMax, Result - and names its output
    ``prob_emotion``, so the values already are a distribution. Feeding them through
    a second softmax compresses them towards uniform: measured on the robot, a
    confident (0.636, 0.096, 0.066, 0.056, 0.146) was published as
    (0.301, 0.176, 0.171, 0.168, 0.184), which reads as an unsure model.

    Values that do not look like a distribution (negative, or not summing to about
    one) are still passed through a softmax, so a blob exporting raw logits keeps
    working.
    """
    values = np.asarray(
        emotion_logits(packet, preferred_layer), dtype=np.float64
    ).reshape(-1)
    if values.size != len(EMOTION_LABELS):
        raise ValueError(
            f"emotion output has {values.size} values, expected {len(EMOTION_LABELS)}"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("emotion output contains a non-finite value")
    total = float(np.sum(values))
    if np.all(values >= 0.0) and abs(total - 1.0) <= 0.05:
        return values / total
    return softmax(values)


def translate_emotion(
    packet,
    face,
    frame_width,
    frame_height,
    output_layer=EMOTION_OUTPUT_LAYER,
):
    """Translate one raw emotion result and its parsed face box."""
    from datatypes.msg import Detection

    probabilities = emotion_probabilities(packet, output_layer)
    winner = int(np.argmax(probabilities))

    detection = Detection()
    detection.label = EMOTION_LABELS[winner]
    detection.score = float(probabilities[winner])
    (
        detection.x_min,
        detection.y_min,
        detection.x_max,
        detection.y_max,
    ) = _face_box_pixels(face, frame_width, frame_height)
    detection.keypoint_names = []
    detection.keypoint_x = []
    detection.keypoint_y = []
    detection.keypoint_z = []
    detection.scalar_names = list(EMOTION_LABELS)
    detection.scalar_values = [float(value) for value in probabilities]
    return detection


def translate_head_pose(packet, face, frame_width, frame_height):
    """Translate the named yaw, pitch, and roll heads for one face crop."""
    from datatypes.msg import Detection

    angles = []
    for scalar_name, layer_name in HEAD_POSE_OUTPUTS:
        try:
            values = np.asarray(packet.getTensor(layer_name), dtype=np.float64).reshape(
                -1
            )
        except (KeyError, RuntimeError) as error:
            raise ValueError(
                f"head-pose output is missing required layer {layer_name}"
            ) from error
        if values.size != 1:
            raise ValueError(
                f"head-pose layer {layer_name} has {values.size} values, expected 1"
            )
        if not np.isfinite(values[0]):
            raise ValueError(
                f"head-pose layer {layer_name} contains a non-finite value"
            )
        angles.append(float(values[0]))

    detection = Detection()
    detection.label = "Face"
    detection.score = 1.0
    (
        detection.x_min,
        detection.y_min,
        detection.x_max,
        detection.y_max,
    ) = _face_box_pixels(face, frame_width, frame_height)
    detection.keypoint_names = []
    detection.keypoint_x = []
    detection.keypoint_y = []
    detection.keypoint_z = []
    detection.scalar_names = [name for name, _ in HEAD_POSE_OUTPUTS]
    detection.scalar_values = angles
    return detection


def facemesh_xyz(packet, preferred_layer=FACEMESH_OUTPUT_LAYER):
    """Read and validate the MediaPipe 468-point XYZ output."""
    try:
        values = packet.getTensor(preferred_layer)
    except (KeyError, RuntimeError):
        names_getter = getattr(packet, "getAllLayerNames", None)
        names = [str(name) for name in names_getter()] if callable(names_getter) else []
        candidates = []
        for name in names:
            tensor = np.asarray(packet.getTensor(name))
            if tensor.size == FACEMESH_LANDMARK_COUNT * 3:
                candidates.append(tensor)
        if len(candidates) != 1:
            raise ValueError(
                "facemesh must expose one 1404-value landmark output; found "
                f"{len(candidates)}"
            )
        values = candidates[0]

    values = np.asarray(values, dtype=np.float32)
    if values.size != FACEMESH_LANDMARK_COUNT * 3:
        raise ValueError(
            f"facemesh output has {values.size} values, expected "
            f"{FACEMESH_LANDMARK_COUNT * 3}"
        )
    values = values.reshape(FACEMESH_LANDMARK_COUNT, 3)
    if not np.all(np.isfinite(values)):
        raise ValueError("facemesh output contains a non-finite value")
    return values


def translate_facemesh(
    packet,
    face,
    frame_width,
    frame_height,
    output_layer=FACEMESH_OUTPUT_LAYER,
):
    """Map crop-space face landmarks back to the published camera frame."""
    from datatypes.msg import Detection

    from .hand_tracking import PalmRegion, map_crop_points_to_frame

    values = np.asarray(facemesh_xyz(packet, output_layer), dtype=np.float64)
    box = face.getBoundingBox()
    # The device now crops a PIXEL square with a fractional padding (see
    # detection_crop_config), so the crop is a rectangle in normalised units. The
    # mapping path carries a single roi_size, interpreted in x-normalised units, so
    # the y component is pre-scaled by the rectangle's own aspect. Without this the
    # published mesh came out about 2.4x too wide (measured against the face box).
    frame_w = float(frame_width)
    frame_h = float(frame_height)
    side_px = max(float(box.size.width) * frame_w, float(box.size.height) * frame_h) * (
        1.0 + 2.0 * FACE_CROP_PADDING
    )
    size_x = side_px / frame_w
    size_y = side_px / frame_h
    region = PalmRegion(
        score=1.0,
        box_x=0.0,
        box_y=0.0,
        box_size=size_x,
        roi_x=float(box.center.x),
        roi_y=float(box.center.y),
        roi_size=size_x,
        rotation=0.0,
    )
    xy_peak = float(np.max(np.abs(values[:, :2])))
    crop_values = np.array(values, copy=True)
    if xy_peak <= 2.0:
        crop_values[:, :2] *= float(FACEMESH_INPUT_SIZE)
    # y is normalised against the crop's height while roi_size carries its width, so
    # the y component is re-scaled AROUND THE CROP CENTRE. Scaling it around zero
    # instead shifted every landmark vertically (caught by the geometry test).
    half = float(FACEMESH_INPUT_SIZE) / 2.0
    crop_values[:, 1] = half + (crop_values[:, 1] - half) * (size_y / size_x)
    mapped = map_crop_points_to_frame(
        crop_values,
        region,
        frame_width,
        frame_height,
        FACEMESH_INPUT_SIZE,
    )

    # The shipped network's XYZ components use one scale. Pixel-space XY means
    # z is in crop pixels too; normalized XY means z is already relative.
    z_scale = 1.0 if xy_peak <= 2.0 else 1.0 / float(FACEMESH_INPUT_SIZE)

    detection = Detection()
    detection.label = "Face"
    detection.score = 1.0
    (
        detection.x_min,
        detection.y_min,
        detection.x_max,
        detection.y_max,
    ) = _face_box_pixels(face, frame_width, frame_height)
    detection.keypoint_names = [
        f"landmark_{index}" for index in range(FACEMESH_LANDMARK_COUNT)
    ]
    detection.keypoint_x = [float(point[0]) for point in mapped]
    detection.keypoint_y = [float(point[1]) for point in mapped]
    detection.keypoint_z = [float(value) * z_scale for value in values[:, 2]]
    detection.scalar_names = []
    detection.scalar_values = []
    return detection


FACE_CROP_TRANSLATORS = {
    "emotion_recognition_lfw_64x64": translate_emotion,
    "facemesh_192x192": translate_facemesh,
    "head-pose-estimation-adas-0001": translate_head_pose,
}


def face_crop_classifier_id(artifact_ids):
    """Return the one supported classifier paired with the face detector."""
    artifact_ids = tuple(artifact_ids)
    if FACE_DETECTOR_MODEL_ID not in artifact_ids:
        raise ValueError("face-crop composite is missing its YuNet detector")
    classifiers = [
        model_id for model_id in artifact_ids if model_id != FACE_DETECTOR_MODEL_ID
    ]
    if len(classifiers) != 1:
        raise ValueError("face-crop composite must contain exactly one classifier")
    classifier_id = classifiers[0]
    if classifier_id not in FACE_CROP_TRANSLATORS:
        raise ValueError(f"unsupported face-crop classifier {classifier_id}")
    return classifier_id
