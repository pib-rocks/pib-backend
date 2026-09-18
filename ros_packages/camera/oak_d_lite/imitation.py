"""Host-side helpers for the official two-stage MediaPipe hand pipeline."""

import math
from typing import Tuple

import depthai as dai

LANDMARK_COUNT = 21
PALM_PADDING = 0.1
LANDMARK_SCORE_THRESHOLD = 0.5


class ProcessDetections(dai.node.HostNode):
    """Turn every palm detection into a timestamp-matched landmark crop config."""

    def __init__(self):
        super().__init__()
        self.detections_input = self.createInput()
        self.config_output = self.createOutput()
        self.padding = PALM_PADDING
        self._target_w = None
        self._target_h = None

    def build(
        self,
        detections_input: dai.Node.Output,
        padding: float,
        target_size: Tuple[int, int],
    ) -> "ProcessDetections":
        self.padding = float(padding)
        self._target_w, self._target_h = target_size
        self.link_args(detections_input)
        return self

    def process(self, img_detections: dai.Buffer) -> None:
        configs = dai.MessageGroup()
        for index, detection in enumerate(img_detections.detections):
            config = detection_crop_config(
                detection,
                self.padding,
                self._target_w,
                self._target_h,
            )
            config.setTimestamp(img_detections.getTimestamp())
            config.setSequenceNum(img_detections.getSequenceNum())
            configs[f"cfg_{index}"] = config

        configs.setTimestamp(img_detections.getTimestamp())
        configs.setSequenceNum(img_detections.getSequenceNum())
        self.config_output.send(configs)


def detection_crop_config(detection, padding, target_width, target_height):
    """Build the padded, rotated STRETCH crop used for one palm."""
    rect = detection.getBoundingBox()
    padded = dai.RotatedRect()
    padded.center.x = rect.center.x
    padded.center.y = rect.center.y
    padded.size.width = rect.size.width + 2.0 * padding
    padded.size.height = rect.size.height + 2.0 * padding
    padded.angle = rect.angle

    config = dai.ImageManipConfig()
    config.addCropRotatedRect(padded, normalizedCoords=True)
    config.setOutputSize(
        target_width,
        target_height,
        dai.ImageManipConfig.ResizeMode.STRETCH,
    )
    config.setReusePreviousImage(False)
    return config


def box_from_points(points, frame_width, frame_height):
    """Enclose mapped frame-pixel landmarks in a non-degenerate bbox."""
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    if not xs or not ys:
        return 0, 0, 1, 1
    x_min = max(0, min(frame_width - 1, int(math.floor(min(xs)))))
    y_min = max(0, min(frame_height - 1, int(math.floor(min(ys)))))
    x_max = max(x_min + 1, min(frame_width, int(math.ceil(max(xs)))))
    y_max = max(y_min + 1, min(frame_height, int(math.ceil(max(ys)))))
    return x_min, y_min, x_max, y_max


def world_landmark_scalars(values):
    """Lay out 21 world XYZ triples as Detection scalar parallel arrays."""
    if len(values) != LANDMARK_COUNT * 3:
        return [], []
    names = []
    scalars = []
    axes = ("x", "y", "z")
    for index in range(LANDMARK_COUNT):
        for axis_index, axis in enumerate(axes):
            value = float(values[index * 3 + axis_index])
            if not math.isfinite(value):
                return [], []
            names.append(f"world_{index}_{axis}")
            scalars.append(value)
    return names, scalars


def _group_value(group, key):
    try:
        return group[key]
    except (KeyError, TypeError):
        return None


def _prediction(message):
    if message is None:
        return None
    try:
        value = float(message.prediction)
    except (AttributeError, IndexError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _keypoint_xyz(message):
    if message is None:
        return []
    try:
        keypoints = message.getKeypoints()
    except AttributeError:
        return []
    values = []
    for keypoint in keypoints:
        coordinates = keypoint.imageCoordinates
        xyz = (float(coordinates.x), float(coordinates.y), float(coordinates.z))
        if not all(math.isfinite(value) for value in xyz):
            return []
        values.append(xyz)
    return values


def _world_values(message):
    points = _keypoint_xyz(message)
    if len(points) == LANDMARK_COUNT:
        return [coordinate for point in points for coordinate in point]
    try:
        predictions = message.predictions
    except AttributeError:
        return []
    values = []
    for prediction in predictions:
        try:
            value = float(prediction.prediction)
        except (AttributeError, TypeError, ValueError):
            return []
        if not math.isfinite(value):
            return []
        values.append(value)
    return values if len(values) == LANDMARK_COUNT * 3 else []


def gathered_hands(
    gathered,
    frame_width,
    frame_height,
    padding=PALM_PADDING,
    score_threshold=LANDMARK_SCORE_THRESHOLD,
):
    """Convert parsed gathered results into the stable imitation hand contract."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")

    detections = list(gathered.reference_data.detections)
    hands = []
    for detection, item in zip(detections, gathered.items):
        rect = detection.getBoundingBox()
        width = float(rect.size.width)
        height = float(rect.size.height)
        xmin = float(rect.center.x) - width / 2.0
        ymin = float(rect.center.y) - height / 2.0
        crop_rect = (
            xmin - padding,
            ymin - padding,
            width + 2.0 * padding,
            height + 2.0 * padding,
        )

        keypoints = _keypoint_xyz(_group_value(item, "0"))
        landmark_score = _prediction(_group_value(item, "1"))
        handedness = _prediction(_group_value(item, "2"))
        if (
            len(keypoints) != LANDMARK_COUNT
            or landmark_score is None
            or landmark_score < score_threshold
        ):
            continue

        points = []
        for x, y, _ in keypoints:
            normalized_x = crop_rect[0] + crop_rect[2] * x
            normalized_y = crop_rect[1] + crop_rect[3] * y
            points.append(
                (
                    min(float(frame_width), max(0.0, normalized_x * frame_width)),
                    min(float(frame_height), max(0.0, normalized_y * frame_height)),
                )
            )

        hands.append(
            {
                "palm_score": float(detection.confidence),
                "landmark_score": landmark_score,
                "handedness": handedness if handedness is not None else 0.0,
                "landmarks": points,
                "world": _world_values(_group_value(item, "3")),
                "crop_rect": crop_rect,
            }
        )
    return hands


def gathered_result_trace_values(gathered, padding=PALM_PADDING):
    """Return palm score, landmark score, and normalized crop for each result."""
    traces = []
    for detection, item in zip(gathered.reference_data.detections, gathered.items):
        rect = detection.getBoundingBox()
        width = float(rect.size.width)
        height = float(rect.size.height)
        traces.append(
            (
                float(detection.confidence),
                _prediction(_group_value(item, "1")),
                (
                    float(rect.center.x) - width / 2.0 - padding,
                    float(rect.center.y) - height / 2.0 - padding,
                    width + 2.0 * padding,
                    height + 2.0 * padding,
                ),
            )
        )
    return traces
