"""Translate parsed DepthAI detections to the backend ROS contract."""


def _pixel(value, extent):
    return min(extent, max(0, int(float(value) * extent)))


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


def translate_detection(parsed, labels, frame_width, frame_height):
    """Translate one normalized parsed detection into pixel coordinates."""
    from datatypes.msg import Detection

    box = parsed.getBoundingBox()
    half_width = float(box.size.width) / 2.0
    half_height = float(box.size.height) / 2.0

    detection = Detection()
    detection.label = _label(parsed, labels)
    detection.score = float(parsed.confidence)
    detection.x_min = _pixel(float(box.center.x) - half_width, frame_width)
    detection.y_min = _pixel(float(box.center.y) - half_height, frame_height)
    detection.x_max = _pixel(float(box.center.x) + half_width, frame_width)
    detection.y_max = _pixel(float(box.center.y) + half_height, frame_height)

    keypoints = _keypoints(parsed)
    detection.keypoint_names = [
        str(getattr(point, "labelName", "") or f"landmark_{index}")
        for index, point in enumerate(keypoints)
    ]
    detection.keypoint_x = [
        float(point.imageCoordinates.x) * frame_width for point in keypoints
    ]
    detection.keypoint_y = [
        float(point.imageCoordinates.y) * frame_height for point in keypoints
    ]
    detection.keypoint_z = [
        float(getattr(point.imageCoordinates, "z", 0.0)) for point in keypoints
    ]
    detection.scalar_names, detection.scalar_values = _scalars(parsed)
    return detection


def translate_detections(packet, labels, frame_width, frame_height):
    """Translate all detections carried by a parsed DepthAI packet."""
    return [
        translate_detection(parsed, labels, frame_width, frame_height)
        for parsed in packet.detections
    ]
