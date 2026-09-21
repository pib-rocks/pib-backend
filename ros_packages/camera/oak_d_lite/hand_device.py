"""Device-side manager for the hand chain: decoding and crop config on the OAK.

The manager Script reads the palm decoder's ``result`` records, computes each
landmark crop as a rotated rect and pushes the ``ImageManipConfig`` straight into
the landmark ImageManip, so no per-frame work crosses the host: the host receives
the assembled hands and publishes them.

Ported from the reference manager Script of the imitation chain.  Differences:
the zoo decoding head returns its TOP 10 records (not top-2), and it emits them as
ten eight-float records without the reference's transposed layout.
"""

from pathlib import Path

_PURE_START = "# -- BEGIN DEVICE PURE FUNCTIONS --"
_PURE_END = "# -- END DEVICE PURE FUNCTIONS --"

# -- BEGIN DEVICE PURE FUNCTIONS --
import math

PALM_SCORE_THRESHOLD = 0.5
LANDMARK_SCORE_THRESHOLD = 0.5
LANDMARK_COUNT = 21
LANDMARK_INPUT_SIZE = 224.0
# The zoo decoding head returns its TOP 10 candidates, eight floats each.
PALM_RECORD_COUNT = 10


def normalize_radians(angle):
    """Normalize an angle to [-pi, pi)."""
    return angle - 2.0 * math.pi * math.floor((angle + math.pi) / (2.0 * math.pi))


def palm_regions(
    values, score_threshold=PALM_SCORE_THRESHOLD, max_hands=PALM_RECORD_COUNT
):
    """Decode and gate the postprocessor's eight-float palm records.

    The head reports PALM_RECORD_COUNT candidates in confidence order; the score
    gate drops the background ones before they cost a landmark crop.
    """
    if len(values) != 8 * PALM_RECORD_COUNT:
        return []
    regions = []
    for offset in range(0, 8 * PALM_RECORD_COUNT, 8):
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
    regions.sort(key=lambda region: region["palm_score"], reverse=True)
    if max_hands > 0:
        regions = regions[:max_hands]
    return regions


def landmark_score_passes(score, threshold=LANDMARK_SCORE_THRESHOLD):
    """Apply the reference landmark-presence threshold."""
    return math.isfinite(float(score)) and float(score) >= threshold


def landmark_pixels_to_square(values, region, source_width, source_height):
    """Map the landmark net's crop pixels back to branch-normalized XYZ.

    The crop is a square in normalized coordinates over a rectangular branch, so
    the offset has to be rotated in PIXELS and only then normalized per axis -
    doing it in normalized coordinates skews the result by the branch aspect.
    The third component stays the hand-relative depth, scaled like x and y.
    """
    if len(values) != LANDMARK_COUNT * 3:
        return []
    if source_width <= 0 or source_height <= 0:
        return []
    rotation = float(region["rotation"])
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    offset_x = float(region["size"]) * source_width
    offset_y = float(region["size"]) * source_height
    points = []
    for index in range(LANDMARK_COUNT):
        u = float(values[index * 3]) / LANDMARK_INPUT_SIZE - 0.5
        v = float(values[index * 3 + 1]) / LANDMARK_INPUT_SIZE - 0.5
        z = float(values[index * 3 + 2]) / LANDMARK_INPUT_SIZE
        if not math.isfinite(u) or not math.isfinite(v) or not math.isfinite(z):
            return []
        # Rotate the crop-space offset back into branch pixels.
        dx = offset_x * (u * cosine + v * sine)
        dy = offset_y * (v * cosine - u * sine)
        points.append(
            (
                float(region["center_x"]) + dx / source_width,
                float(region["center_y"]) + dy / source_height,
                z,
            )
        )
    return points


def fit_landmark_region(region, source_width, source_height, inset=0.5):
    """Shrink the square ROI until ImageManip can validate it.

    Mirrors the host chain's ``_landmark_crop_config``: the fit happens in BRANCH
    PIXELS, per axis, with the rotation folded in.  The palm ImageManip warps the
    branch's full rectangle onto the square network input without letterboxing,
    so the decoder's normalized axes are the branch axes - treating them as a
    letterboxed square would place the rect off the image, and an invalid crop
    makes the manip drop the frame, which leaves the Script waiting forever and
    trips the device watchdog.
    """
    if source_width <= 0 or source_height <= 0:
        return None
    inset = max(0.0, min(float(inset), source_width / 4.0, source_height / 4.0))
    size = float(region["size"])
    rotation = float(region["rotation"])
    width_px = size * source_width
    height_px = size * source_height
    if width_px <= 0.0 or height_px <= 0.0:
        return None
    extent_x = 0.5 * (
        width_px * abs(math.cos(rotation)) + height_px * abs(math.sin(rotation))
    )
    extent_y = 0.5 * (
        width_px * abs(math.sin(rotation)) + height_px * abs(math.cos(rotation))
    )
    center_x = min(
        source_width - inset, max(inset, float(region["center_x"]) * source_width)
    )
    center_y = min(
        source_height - inset, max(inset, float(region["center_y"]) * source_height)
    )
    available_x = max(inset, min(center_x, source_width - center_x) - inset)
    available_y = max(inset, min(center_y, source_height - center_y) - inset)
    if extent_x <= 0.0 or extent_y <= 0.0 or available_x <= 0.0 or available_y <= 0.0:
        return None
    scale = min(1.0, available_x / extent_x, available_y / extent_y)
    fitted = dict(region)
    fitted["center_x"] = center_x / source_width
    fitted["center_y"] = center_y / source_height
    fitted["size"] = size * scale
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


def build_hand_script(source_width=256, source_height=144):
    """Build the per-frame manager Script that runs on the OAK device."""
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

def layer_reader(packet):
    # Inside a Script node the firmware's own 2.x-era bindings apply: getLayerFp16
    # is the device reader, getTensor belongs to the host library.  Resolve both
    # spellings and fail loudly - a silent miss would only yield empty detections.
    reader = getattr(packet, "getLayerFp16", None)
    if reader is None:
        reader = getattr(packet, "getTensor", None)
    if reader is None:
        raise RuntimeError("packet exposes neither getLayerFp16 nor getTensor")
    return reader


def flat_tensor(packet, name):
    return flatten_values(layer_reader(packet)(name))

def palm_tensor(packet):
    tensor = layer_reader(packet)("result")
    if tensor is None:
        return []
    return flatten_values(tensor)

def palm_config():
    config = ImageManipConfig()
    # STRETCH matches the host mapping: the palm input is the branch warped to a
    # square, so each normalised axis maps straight onto the published frame.
    config.setOutputSize(PD_SIZE, PD_SIZE, ImageManipConfig.ResizeMode.STRETCH)
    config.setFrameType(ImgFrame.Type.BGR888p)
    return config

def landmark_config(region, reuse):
    rotated = RotatedRect()
    rotated.center.x = region["center_x"]
    rotated.center.y = region["center_y"]
    # Square ROI, but the branch axes scale differently - keep it square in
    # normalized coordinates the way the host path does.
    rotated.size.width = region["size"]
    rotated.size.height = region["size"]
    rotated.angle = math.degrees(region["rotation"])
    config = ImageManipConfig()
    # The device binding only accepts setOutputSize(w, h, ResizeMode).  The crop is
    # square and so is the output, so LETTERBOX adds no padding.
    config.setOutputSize(LM_SIZE, LM_SIZE, ImageManipConfig.ResizeMode.LETTERBOX)
    config.setFrameType(ImgFrame.Type.BGR888p)
    config.addCropRotatedRect(rotated, True)
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
        fitted_region = fit_landmark_region(region, SOURCE_WIDTH, SOURCE_HEIGHT)
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
        square_points = landmark_pixels_to_square(
            image_values, fitted_region, SOURCE_WIDTH, SOURCE_HEIGHT
        )
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
                # The host rebuilds its PalmRegion from these, so the published
                # box stays the decoder's palm box.
                "center_x": region["center_x"],
                "center_y": region["center_y"],
                "size": region["size"],
                "rotation": region["rotation"],
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
