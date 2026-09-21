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
