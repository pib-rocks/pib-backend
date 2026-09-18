"""Pure geometry and device Script for the on-device imitation hand chain."""

from pathlib import Path

_PURE_START = "# -- BEGIN DEVICE PURE FUNCTIONS --"
_PURE_END = "# -- END DEVICE PURE FUNCTIONS --"

# -- BEGIN DEVICE PURE FUNCTIONS --
import math

PALM_SCORE_THRESHOLD = 0.5
LANDMARK_SCORE_THRESHOLD = 0.5
LANDMARK_COUNT = 21
LANDMARK_INPUT_SIZE = 224.0


def normalize_radians(angle):
    """Normalize an angle to [-pi, pi)."""
    return angle - 2.0 * math.pi * math.floor((angle + math.pi) / (2.0 * math.pi))


def palm_regions(values, score_threshold=PALM_SCORE_THRESHOLD):
    """Decode and gate the postprocessor's two eight-float palm records."""
    if len(values) != 16:
        return []
    regions = []
    for offset in (0, 8):
        record = values[offset : offset + 8]
        if len(record) != 8 or not all(math.isfinite(float(value)) for value in record):
            continue
        score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y = [
            float(value) for value in record
        ]
        if score < score_threshold or box_size <= 0.0:
            continue
        rotation = normalize_radians(
            0.5 * math.pi - math.atan2(-(kp2_y - kp0_y), kp2_x - kp0_x)
        )
        regions.append(
            {
                "palm_score": score,
                "box_x": box_x,
                "box_y": box_y,
                "box_size": box_size,
                "center_x": box_x + 0.5 * box_size * math.sin(rotation),
                "center_y": box_y - 0.5 * box_size * math.cos(rotation),
                "size": 2.9 * box_size,
                "rotation": rotation,
            }
        )
    return regions


def landmark_score_passes(score, threshold=LANDMARK_SCORE_THRESHOLD):
    """Apply the reference landmark-presence threshold."""
    return math.isfinite(float(score)) and float(score) >= threshold


def landmark_pixels_to_square(values, region):
    """Map 21 landmark-crop XYZ triples into square-normalized XY points."""
    if len(values) != LANDMARK_COUNT * 3:
        return []
    rotation = region["rotation"]
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    points = []
    for index in range(LANDMARK_COUNT):
        x = float(values[index * 3]) / LANDMARK_INPUT_SIZE
        y = float(values[index * 3 + 1]) / LANDMARK_INPUT_SIZE
        if not math.isfinite(x) or not math.isfinite(y):
            return []
        points.append(
            (
                region["center_x"]
                + region["size"] * ((x - 0.5) * cosine + (0.5 - y) * sine),
                region["center_y"]
                + region["size"] * ((y - 0.5) * cosine + (x - 0.5) * sine),
            )
        )
    return points


def fit_landmark_region(region, frame_width, frame_height, inset=0.5):
    """Shrink a rotated square around its center until DepthAI can validate it."""
    pad_h = (frame_width - frame_height) // 2
    center_x = float(region["center_x"]) * frame_width
    center_y = float(region["center_y"]) * frame_width - pad_h
    size = float(region["size"]) * frame_width
    rotation = float(region["rotation"])
    extent = 0.5 * size * (abs(math.cos(rotation)) + abs(math.sin(rotation)))
    available_x = min(center_x, frame_width - center_x) - inset
    available_y = min(center_y, frame_height - center_y) - inset
    if extent <= 0.0 or available_x <= 0.0 or available_y <= 0.0:
        return None
    scale = min(1.0, available_x / extent, available_y / extent)
    fitted = dict(region)
    fitted["size"] = float(region["size"]) * scale
    return fitted


def square_to_frame(point, frame_width, frame_height):
    """Undo landscape square letterboxing and return clamped frame pixels."""
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")
    pad_h = (frame_width - frame_height) // 2
    x = float(point[0]) * frame_width
    y = float(point[1]) * frame_width - pad_h
    return (
        min(float(frame_width), max(0.0, x)),
        min(float(frame_height), max(0.0, y)),
    )


def square_points_to_frame(points, frame_width, frame_height):
    return [square_to_frame(point, frame_width, frame_height) for point in list(points)]


def box_from_points(points, frame_width, frame_height):
    """Enclose already mapped frame-pixel landmarks in a non-degenerate bbox.

    The palm box the detector emits covers the palm only, so fingers stick out
    of it; a viewer reads that as a bug. Deriving the box from the landmarks
    cannot disagree with the points that are drawn next to it.
    """
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    if not xs or not ys:
        return 0, 0, 1, 1
    x_min = max(0, min(frame_width - 1, int(math.floor(min(xs)))))
    y_min = max(0, min(frame_height - 1, int(math.floor(min(ys)))))
    x_max = max(x_min + 1, min(frame_width, int(math.ceil(max(xs)))))
    y_max = max(y_min + 1, min(frame_height, int(math.ceil(max(ys)))))
    return x_min, y_min, x_max, y_max


def square_box_to_frame(region, frame_width, frame_height):
    """Map the detector square bbox to a non-degenerate frame-pixel bbox."""
    half = 0.5 * float(region["box_size"])
    low = square_to_frame(
        (region["box_x"] - half, region["box_y"] - half),
        frame_width,
        frame_height,
    )
    high = square_to_frame(
        (region["box_x"] + half, region["box_y"] + half),
        frame_width,
        frame_height,
    )
    x_min = max(0, min(frame_width - 1, int(math.floor(min(low[0], high[0])))))
    y_min = max(0, min(frame_height - 1, int(math.floor(min(low[1], high[1])))))
    x_max = max(x_min + 1, min(frame_width, int(math.ceil(max(low[0], high[0])))))
    y_max = max(y_min + 1, min(frame_height, int(math.ceil(max(low[1], high[1])))))
    return x_min, y_min, x_max, y_max


def world_landmark_scalars(values):
    """Lay out 21 world XYZ triples as Detection scalar parallel arrays."""
    if len(values) != LANDMARK_COUNT * 3:
        return [], []
    names = []
    scalars = []
    axes = ("x", "y", "z")
    for index in range(LANDMARK_COUNT):
        for axis_index in range(3):
            value = float(values[index * 3 + axis_index])
            if not math.isfinite(value):
                return [], []
            names.append("world_%d_%s" % (index, axes[axis_index]))
            scalars.append(value)
    return names, scalars


# -- END DEVICE PURE FUNCTIONS --


def device_pure_source():
    """Return the exact pure-function source embedded in the device Script."""
    source = Path(__file__).read_text(encoding="utf-8")
    start = source.index(_PURE_START, source.index(_PURE_START) + 1)
    end = source.index(_PURE_END, start) + len(_PURE_END)
    return source[start:end]


def build_imitation_script(source_width=256, source_height=144):
    """Build the no-tracking per-frame manager Script used by DepthAI."""
    loop = r"""
import marshal

SOURCE_WIDTH = ${SOURCE_WIDTH}
SOURCE_HEIGHT = ${SOURCE_HEIGHT}
PD_SIZE = 128
LM_SIZE = 224

def flatten_values(value):
    if value is None:
        return []
    try:
        return [float(value)]
    except (TypeError, ValueError):
        pass
    values = []
    for item in value:
        values.extend(flatten_values(item))
    return values

def read_layer(packet, name):
    # The on-device Script runtime exposes lpb.NNData, which offers getLayerFp16
    # and has no getTensor; the host-side depthai 3.x NNData is the other way
    # round. Prefer the device API so the embedded copy of this module runs, and
    # fall back to the host API so the same source stays unit-testable.
    reader = getattr(packet, "getLayerFp16", None)
    if reader is not None:
        return reader(name)
    return packet.getTensor(name)

def flat_tensor(packet, name):
    return flatten_values(read_layer(packet, name))

def palm_tensor(packet):
    tensor = read_layer(packet, "result")
    if tensor is None:
        return []
    if len(tensor) == 8:
        try:
            if len(tensor[0]) == 2:
                return [
                    float(tensor[feature][detection])
                    for detection in range(2)
                    for feature in range(8)
                ]
        except (TypeError, IndexError):
            pass
    return flatten_values(tensor)

def palm_config():
    config = ImageManipConfig()
    config.setOutputSize(PD_SIZE, PD_SIZE, ImageManipConfig.ResizeMode.LETTERBOX)
    config.setFrameType(ImgFrame.Type.BGR888p)
    return config

def landmark_config(region, reuse):
    pad_h = (SOURCE_WIDTH - SOURCE_HEIGHT) // 2
    rotated = RotatedRect()
    rotated.center.x = region["center_x"]
    rotated.center.y = (
        region["center_y"] * SOURCE_WIDTH - pad_h
    ) / float(SOURCE_HEIGHT)
    rotated.size.width = region["size"]
    rotated.size.height = region["size"] * SOURCE_WIDTH / float(SOURCE_HEIGHT)
    rotated.angle = math.degrees(region["rotation"])
    config = ImageManipConfig()
    # The Script runtime requires the resize mode argument; the two-argument
    # host overload does not exist there. The rotated ROI is already square, so
    # STRETCH is the faithful equivalent of the reference's setResize call.
    config.setOutputSize(LM_SIZE, LM_SIZE, ImageManipConfig.ResizeMode.STRETCH)
    config.setFrameType(ImgFrame.Type.BGR888p)
    # depthai 3.x named this addCropRotatedRect; the 2.x-era Script runtime on
    # the device may still only offer setCropRotatedRect. Try both so one
    # embedded source runs in either runtime.
    crop = getattr(config, "addCropRotatedRect", None)
    if crop is None:
        crop = getattr(config, "setCropRotatedRect", None)
    crop(rotated, True)
    border_replicate = getattr(config, "setWarpBorderReplicatePixels", None)
    if border_replicate is not None:
        border_replicate()
    config.setReusePreviousImage(reuse)
    return config

stages = {
    "palm_detector_nn": 0,
    "decoding_nn": 0,
    "decoding_result": 0,
    "image_manip_config": 0,
    "image_manip_roi": 0,
    "hand_landmark_nn": 0,
    "post_processing": 0,
    "publish": 0,
}

while True:
    node.io["pre_pd_manip_cfg"].send(palm_config())
    palm_packet = node.io["from_post_pd_nn"].get()
    stages["palm_detector_nn"] += 1
    stages["decoding_nn"] += 1
    palm_values = palm_tensor(palm_packet)
    regions = palm_regions(palm_values)
    stages["decoding_result"] += 1
    hands = []
    fitted_regions = []
    for region in regions:
        fitted_region = fit_landmark_region(
            region, SOURCE_WIDTH, SOURCE_HEIGHT
        )
        if fitted_region is None:
            stages["post_processing"] += 1
            continue
        fitted_regions.append((region, fitted_region))
    for index in range(len(fitted_regions)):
        region, fitted_region = fitted_regions[index]
        node.io["pre_lm_manip_cfg"].send(
            landmark_config(fitted_region, index + 1 < len(fitted_regions))
        )
        stages["image_manip_config"] += 1
        landmark_packet = node.io["from_lm_nn"].get()
        stages["image_manip_roi"] += 1
        stages["hand_landmark_nn"] += 1
        score_values = flat_tensor(landmark_packet, "Identity_1")
        if len(score_values) != 1 or not landmark_score_passes(score_values[0]):
            stages["post_processing"] += 1
            continue
        image_values = flat_tensor(
            landmark_packet, "Identity_dense/BiasAdd/Add"
        )
        square_points = landmark_pixels_to_square(image_values, fitted_region)
        if len(square_points) != LANDMARK_COUNT:
            stages["post_processing"] += 1
            continue
        handedness_values = flat_tensor(landmark_packet, "Identity_2")
        world_values = flat_tensor(
            landmark_packet, "Identity_3_dense/BiasAdd/Add"
        )
        hands.append(
            {
                "palm_score": region["palm_score"],
                "landmark_score": float(score_values[0]),
                "handedness": (
                    float(handedness_values[0]) if handedness_values else 0.0
                ),
                "box_x": region["box_x"],
                "box_y": region["box_y"],
                "box_size": region["box_size"],
                "landmarks": square_points,
                "world": world_values,
            }
        )
        stages["post_processing"] += 1
    stages["publish"] += 1
    data = marshal.dumps({"hands": hands, "stages": stages})
    output = Buffer(len(data))
    output.setData(data)
    node.io["host"].send(output)
"""
    return (
        device_pure_source()
        + "\n"
        + loop.replace("${SOURCE_WIDTH}", str(int(source_width))).replace(
            "${SOURCE_HEIGHT}", str(int(source_height))
        )
    )
