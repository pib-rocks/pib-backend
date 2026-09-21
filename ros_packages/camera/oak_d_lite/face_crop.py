"""Host-side translation helpers for face-crop classifier chains."""

import math

import numpy as np

EMOTION_LABELS = ("neutral", "happy", "sad", "surprise", "anger")
EMOTION_OUTPUT_LAYER = "prob_emotion"
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
    box = face.getBoundingBox()
    half_width = float(box.size.width) / 2.0
    half_height = float(box.size.height) / 2.0

    detection = Detection()
    detection.label = EMOTION_LABELS[winner]
    detection.score = float(probabilities[winner])
    detection.x_min = _pixel(float(box.center.x) - half_width, frame_width)
    detection.y_min = _pixel(float(box.center.y) - half_height, frame_height)
    detection.x_max = _pixel(float(box.center.x) + half_width, frame_width)
    detection.y_max = _pixel(float(box.center.y) + half_height, frame_height)
    detection.keypoint_names = []
    detection.keypoint_x = []
    detection.keypoint_y = []
    detection.keypoint_z = []
    detection.scalar_names = list(EMOTION_LABELS)
    detection.scalar_values = [float(value) for value in probabilities]
    return detection
