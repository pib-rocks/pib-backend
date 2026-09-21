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


def translate_emotion(
    packet,
    face,
    frame_width,
    frame_height,
    output_layer=EMOTION_OUTPUT_LAYER,
):
    """Translate one raw emotion result and its parsed face box."""
    from datatypes.msg import Detection

    probabilities = softmax(emotion_logits(packet, output_layer))
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

    values = facemesh_xyz(packet, output_layer)
    box = face.getBoundingBox()
    crop_size = (
        max(float(box.size.width), float(box.size.height)) + 2.0 * FACE_CROP_PADDING
    )
    region = PalmRegion(
        score=1.0,
        box_x=0.0,
        box_y=0.0,
        box_size=max(float(box.size.width), float(box.size.height)),
        roi_x=float(box.center.x),
        roi_y=float(box.center.y),
        roi_size=crop_size,
        rotation=0.0,
    )
    xy_peak = float(np.max(np.abs(values[:, :2])))
    crop_values = np.array(values, copy=True)
    if xy_peak <= 2.0:
        crop_values[:, :2] *= float(FACEMESH_INPUT_SIZE)
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
