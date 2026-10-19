"""Translate parsed DepthAI detections to the backend ROS contract."""


def map_fraction(fraction, origin, size):
    """Map a fraction of one axis of the branch field into published pixels.

    ``fraction`` is 0 at the start of the field the branch actually covers and
    1 at its end. The published coordinate is ``origin + fraction * size``.
    """
    return float(origin) + float(fraction) * float(size)


def published_field(origin, far, preview_size, published_size):
    """Turn the branch's corners in the preview into a published-frame field.

    ``origin`` and ``far`` are the branch image's top-left and bottom-right
    in preview pixels. ``preview_size`` is that preview; ``published_size`` is
    the frame the UI displays, which can be a later resize of the preview.
    The result is ``(origin_x, size_x, origin_y, size_y)``.
    """
    preview_width, preview_height = preview_size
    published_width, published_height = published_size
    if (
        preview_width <= 0
        or preview_height <= 0
        or published_width <= 0
        or published_height <= 0
    ):
        raise ValueError("frame size is not positive")
    scale_x = float(published_width) / float(preview_width)
    scale_y = float(published_height) / float(preview_height)
    origin_x = float(origin[0]) * scale_x
    origin_y = float(origin[1]) * scale_y
    size_x = (float(far[0]) - float(origin[0])) * scale_x
    size_y = (float(far[1]) - float(origin[1])) * scale_y
    return (origin_x, size_x, origin_y, size_y)


def _pixel(fraction, origin, size, frame_extent):
    return min(int(frame_extent), max(0, int(map_fraction(fraction, origin, size))))


def _label(parsed, labels):
    label_name = str(getattr(parsed, "labelName", "") or "")
    if label_name:
        return label_name
    label_index = int(getattr(parsed, "label", 0))
    if 0 <= label_index < len(labels):
        return labels[label_index]
    return str(label_index)


def _keypoints(parsed):
    getter = getattr(parsed, "getKeypoints", None)
    return list(getter()) if callable(getter) else []


def _scalars(parsed):
    names = list(getattr(parsed, "scalar_names", ()))
    values = [float(value) for value in getattr(parsed, "scalar_values", ())]
    if len(names) != len(values):
        raise ValueError("parsed scalar names and values have different lengths")
    return names, values


def translate_detection(parsed, labels, frame_width, frame_height, field=None):
    """Translate one normalized parsed detection into pixel coordinates.

    ``field`` is ``(origin_x, size_x, origin_y, size_y)`` in published-frame
    pixels: the window the colour branch covers. A fraction of that window
    maps to ``origin + fraction * size``. Omitting it maps each fraction onto
    the full published frame.
    """
    from datatypes.msg import Detection

    if field is None:
        field = (0.0, float(frame_width), 0.0, float(frame_height))
    origin_x, size_x, origin_y, size_y = field

    box = parsed.getBoundingBox()
    half_width = float(box.size.width) / 2.0
    half_height = float(box.size.height) / 2.0
    x_min = float(box.center.x) - half_width
    y_min = float(box.center.y) - half_height
    x_max = float(box.center.x) + half_width
    y_max = float(box.center.y) + half_height

    detection = Detection()
    detection.label = _label(parsed, labels)
    detection.score = float(parsed.confidence)
    detection.x_min = _pixel(x_min, origin_x, size_x, frame_width)
    detection.y_min = _pixel(y_min, origin_y, size_y, frame_height)
    detection.x_max = _pixel(x_max, origin_x, size_x, frame_width)
    detection.y_max = _pixel(y_max, origin_y, size_y, frame_height)

    keypoints = _keypoints(parsed)
    detection.keypoint_names = [
        str(getattr(point, "labelName", "") or f"landmark_{index}")
        for index, point in enumerate(keypoints)
    ]
    detection.keypoint_x = [
        map_fraction(point.imageCoordinates.x, origin_x, size_x) for point in keypoints
    ]
    detection.keypoint_y = [
        map_fraction(point.imageCoordinates.y, origin_y, size_y) for point in keypoints
    ]
    detection.keypoint_z = [
        float(getattr(point.imageCoordinates, "z", 0.0)) for point in keypoints
    ]
    detection.scalar_names, detection.scalar_values = _scalars(parsed)
    return detection


def translate_detections(packet, labels, frame_width, frame_height, field=None):
    """Translate all detections carried by a parsed DepthAI packet."""
    return [
        translate_detection(parsed, labels, frame_width, frame_height, field)
        for parsed in packet.detections
    ]
