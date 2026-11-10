"""Device-free tests for hand decoder and landmark coordinate handling."""

import ast
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from ros_packages.camera.oak_d_lite.hand_tracking import (
    FAST_BRANCH_HEIGHT,
    FAST_BRANCH_WIDTH,
    FAST_DECODER_INPUT,
    FAST_FP16_LAYERS,
    FAST_LANDMARK_CONFIG_OUTPUT,
    FAST_LANDMARK_INPUT,
    FAST_PALM_CONFIG_OUTPUT,
    FAST_RESULT_OUTPUT,
    PalmRegion,
    build_fast_tracker_script,
    decode_palm_result,
    fit_manip_crop,
    landmark_score,
    landmark_xyz,
    letterbox_square,
    map_fast_hand,
    map_landmarks_to_frame,
    parse_fast_script_result,
    prerotated_pixel_crop_rect,
    relative_landmark_z,
    validate_fast_branch_size,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _message_fields(name):
    path = REPO_ROOT / "ros_packages/datatypes/msg" / name
    return [
        line.split("#", 1)[0].strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.split("#", 1)[0].strip()
    ]


def test_detection_messages_preserve_pixel_and_depth_contract():
    assert _message_fields("Detection.msg") == [
        "string label",
        "float32 score",
        "int32 x_min",
        "int32 y_min",
        "int32 x_max",
        "int32 y_max",
        "string[] keypoint_names",
        "float32[] keypoint_x",
        "float32[] keypoint_y",
        "float32[] keypoint_z",
        "string[] scalar_names",
        "float32[] scalar_values",
    ]
    assert _message_fields("DetectionArray.msg") == [
        "std_msgs/Header header",
        "string model_id",
        "uint32 frame_width",
        "uint32 frame_height",
        "Detection[] detections",
    ]


def test_decoder_maps_a_postprocessing_record_and_bbox_to_full_frame():
    tensor = np.zeros((10, 8), dtype=np.float32)
    tensor[0] = [0.9, 0.5, 0.5, 0.2, 0.5, 0.6, 0.5, 0.4]

    palms = decode_palm_result(tensor)

    assert len(palms) == 1
    assert palms[0].score == pytest.approx(0.9)
    assert palms[0].rotation == pytest.approx(0.0)
    assert palms[0].bbox_pixels(1000, 500) == (400, 200, 600, 300)


def test_bbox_maps_from_warped_nn_branch_to_preview_frame():
    palm = PalmRegion(0.9, 0.5, 0.25, 0.0, 0.5, 0.25, 0.2, 0.0)

    # ImageManip warps the complete rectangular branch to the square network
    # input, so each normalized decoder axis maps directly to the output axis.
    assert palm.bbox_pixels(
        frame_width=640,
        frame_height=480,
        source_width=2104,
        source_height=1560,
    ) == (320, 120, 320, 120)


def test_palm_roi_uses_warped_branch_normalized_coordinates():
    palm = PalmRegion(0.9, 0.5, 0.25, 0.2, 0.6, 0.3, 0.4, 0.0)

    assert palm.roi_for_frame(2104, 1560) == (0.6, 0.3, 0.4, 0.4)


def test_decoder_orders_the_ten_records_by_score_and_honours_the_cap():
    """Ten records arrive; the cap decides how many become a landmark crop.

    The zoo decoding head returns its TOP 10 candidates, most of them
    background.  Their order in the layer is the head's confidence order, the
    decoder re-sorts by score anyway, and it must never invent records.
    """
    tensor = np.zeros((10, 8), dtype=np.float32)
    tensor[0] = [0.6, 0.5, 0.5, 0.2, 0.5, 0.6, 0.5, 0.4]
    tensor[1] = [0.95, 0.25, 0.25, 0.2, 0.25, 0.35, 0.25, 0.15]

    palms = decode_palm_result(tensor)
    assert [round(palm.score, 3) for palm in palms] == [0.95, 0.6]

    best_only = decode_palm_result(tensor, max_hands=1)
    assert [round(palm.score, 3) for palm in best_only] == [0.95]

    every = decode_palm_result(tensor, max_hands=0)
    assert len(every) == 2


def test_decoder_returns_empty_for_no_confident_hand():
    assert decode_palm_result(np.zeros(80, dtype=np.float32)) == []


def test_decoder_rejects_wrong_shape():
    with pytest.raises(ValueError, match="exactly 80"):
        decode_palm_result(np.zeros(79, dtype=np.float32))


def test_landmarks_rotate_scale_and_map_to_actual_full_frame():
    palm = PalmRegion(
        score=0.9,
        box_x=0.5,
        box_y=0.5,
        box_size=0.2,
        roi_x=0.5,
        roi_y=0.5,
        roi_size=0.5,
        rotation=0.0,
    )
    tensor = np.tile([112.0, 112.0, 7.0], (21, 1))
    tensor[1, :2] = [224.0, 112.0]

    points = map_landmarks_to_frame(tensor, palm, 2104, 1560)

    assert len(points) == 21
    assert points[0] == pytest.approx((1052.0, 780.0))
    assert points[1] == pytest.approx((1578.0, 780.0))


def test_landmarks_apply_roi_rotation():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.4, math.pi / 2)
    tensor = np.tile([224.0, 112.0, 0.0], (21, 1))

    points = map_landmarks_to_frame(tensor, palm, 1000, 500)

    assert points[0] == pytest.approx((500.0, 350.0))


def test_landmarks_empty_tensor_returns_empty():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.4, 0.0)
    assert map_landmarks_to_frame([], palm, 100, 100) == []


def test_landmarks_reject_wrong_shape():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.4, 0.0)
    with pytest.raises(ValueError, match="exactly 63"):
        map_landmarks_to_frame(np.zeros(62), palm, 100, 100)


def test_landmark_score_reads_batched_singleton():
    assert landmark_score(np.array([[0.91]], dtype=np.float32)) == pytest.approx(0.91)


def test_landmark_xyz_reads_batched_vector():
    xyz = landmark_xyz(np.arange(63, dtype=np.float32).reshape(1, 63))

    assert xyz.shape == (21, 3)
    assert xyz[0].tolist() == [0.0, 1.0, 2.0]


def test_normalized_landmarks_map_the_same_as_crop_pixels():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.5, 0.0)
    pixels = np.tile([112.0, 112.0, 0.0], (21, 1))
    normalized = np.tile([0.5, 0.5, 0.0], (21, 1))

    pixel_points = map_landmarks_to_frame(pixels, palm, 2104, 1560)
    assert map_landmarks_to_frame(normalized, palm, 2104, 1560) == pixel_points


def test_manip_crop_edges_stay_inside_the_measured_source():
    crop = fit_manip_crop(200, 150, 128, 128)

    assert (crop.output_width, crop.output_height) == (128, 128)
    assert (crop.center_x, crop.center_y) == (0.5, 0.5)
    assert (crop.center_x - crop.width / 2.0) * 200 == pytest.approx(0.5)
    assert (crop.center_x + crop.width / 2.0) * 200 == pytest.approx(199.5)
    assert (crop.center_y - crop.height / 2.0) * 150 == pytest.approx(0.5)
    assert (crop.center_y + crop.height / 2.0) * 150 == pytest.approx(149.5)


def test_manip_crop_preserves_the_full_measured_source():
    crop = fit_manip_crop(640, 480, 128, 128)

    crop_width = crop.width * 640
    crop_height = crop.height * 480
    assert crop_width == pytest.approx(639.0)
    assert crop_height == pytest.approx(479.0)
    assert crop.center_x - crop.width / 2.0 > 0.0
    assert crop.center_y + crop.height / 2.0 < 1.0


def test_manip_crop_follows_a_changed_branch_size():
    crop = fit_manip_crop(1280, 720, 224, 224)

    assert crop.width * 1280 == pytest.approx(1279.0)
    assert crop.height * 720 == pytest.approx(719.0)
    assert (crop.output_width, crop.output_height) == (224, 224)


def test_manip_crop_always_targets_the_network_input_size():
    # A smaller frame would not match the tensor the model expects, so a source
    # below the network input is upscaled instead of trimming the target.
    crop = fit_manip_crop(64, 48, 224, 224)

    assert (crop.output_width, crop.output_height) == (224, 224)
    assert crop.width * 64 == pytest.approx(63.0)
    assert crop.height * 48 == pytest.approx(47.0)


def test_manip_crop_rejects_degenerate_dimensions():
    with pytest.raises(ValueError, match="source must have positive"):
        fit_manip_crop(0, 480, 128, 128)
    with pytest.raises(ValueError, match="output must have positive"):
        fit_manip_crop(640, 480, 128, 0)


def test_relative_landmark_z_keeps_the_scale_of_x_and_y():
    """The reference divides every landmark component by the input size.

    ``rrn_lms[3*i+2] /= lm_input_size`` in HandTrackerEdge puts the relative
    depth in the same unitless scale as x and y, and the finger angles are
    computed from those three-component vectors.  A crop-space tensor already
    normalised to 0..1 must therefore come through untouched, while a tensor in
    landmark-input pixels is scaled like the pixels are.
    """
    normalized = np.zeros((1, 63), dtype=np.float32)
    for i in range(21):
        normalized[0, 3 * i] = 0.4
        normalized[0, 3 * i + 1] = 0.6
        normalized[0, 3 * i + 2] = -0.05 * (i + 1)
    values = relative_landmark_z(normalized, 224)
    assert len(values) == 21
    assert values[0] == pytest.approx(-0.05)
    assert values[20] == pytest.approx(-1.05)

    in_pixels = np.zeros((1, 63), dtype=np.float32)
    for i in range(21):
        in_pixels[0, 3 * i] = 112.0
        in_pixels[0, 3 * i + 1] = 112.0
        in_pixels[0, 3 * i + 2] = 44.8
    values = relative_landmark_z(in_pixels, 224)
    assert values[0] == pytest.approx(0.2)


def test_relative_landmark_z_preserves_the_sign_of_the_relative_depth():
    """The z is signed; clamping it into 0..1 would destroy it.

    ``depthai_nodes``' keypoint parser does ``np.clip(keypoints, 0, 1)``, which
    is right for the image coordinates and fatal for a relative depth that sits
    below the wrist.  This is the function that must not do that.
    """
    tensor = np.zeros((1, 63), dtype=np.float32)
    tensor[0, 2] = -0.3
    tensor[0, 5] = 0.3
    values = relative_landmark_z(tensor, 224)
    assert values[0] == pytest.approx(-0.3)
    assert values[1] == pytest.approx(0.3)


def test_relative_landmark_z_is_empty_without_landmarks():
    assert relative_landmark_z(np.zeros((0,), dtype=np.float32), 224) == []


def test_decoder_accepts_the_top2_postprocessing_layout():
    top2 = np.zeros((2, 8), dtype=np.float32)
    top2[0] = [0.8, 0.4, 0.4, 0.2, 0.4, 0.5, 0.4, 0.3]
    zoo = np.zeros((10, 8), dtype=np.float32)
    zoo[0] = top2[0]

    from_top2 = decode_palm_result(top2)
    from_zoo = decode_palm_result(zoo)

    assert len(from_top2) == 1
    assert from_top2[0] == from_zoo[0]


def test_fast_branch_stays_inside_the_palm_warp_limit():
    # The branch the reference implementation proved (1152x640 preview) is the
    # ceiling; the ISP frame itself is refused.
    assert validate_fast_branch_size(1152, 648) == pytest.approx(9.0)
    assert validate_fast_branch_size(256, 144) == pytest.approx(2.0)
    with pytest.raises(ValueError, match="WARP_SWCH_ERR_CACHE_TOO_SMALL"):
        validate_fast_branch_size(2104, 1560)
    with pytest.raises(ValueError, match="must stay within"):
        validate_fast_branch_size(1920, 1080)


# Callables present on device lpb.NNData. Host-only names such as getTensor are absent.
_DEVICE_NNDATA_METHODS = frozenset(
    {
        "getAllLayerNames",
        "getAllLayers",
        "getData",
        "getFirstLayerFp16",
        "getFirstLayerInt32",
        "getFirstLayerUInt8",
        "getLayerFp16",
        "getLayerInt32",
        "getLayerUInt8",
        "getSequenceNum",
        "getTimestamp",
        "getTransformation",
        "hasLayer",
        "setData",
        "setLayer",
        "setSequenceNum",
        "setTimestamp",
        "setTransformation",
    }
)


def _is_nndata_method_name(name):
    if name in _DEVICE_NNDATA_METHODS or name in {
        "getTensor",
        "getTensorInfo",
        "getTensorDatatype",
        "getFirstTensor",
        "getLayerDatatype",
    }:
        return True
    return name.startswith(
        ("getTensor", "getLayer", "getFirstLayer", "getFirstTensor", "getAllLayer")
    ) or name in {"hasLayer", "addTensor"}


def _nndata_method_calls(script):
    calls = []
    for node in ast.walk(ast.parse(script)):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if _is_nndata_method_name(node.func.attr):
            calls.append(node)
    return calls


def _tensor_values_function(script):
    tree = ast.parse(script)
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "tensor_values"
    )
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, "<tensor_values>", "exec"), namespace)
    return namespace["tensor_values"]


class _DeviceNnData:
    """Stand-in for lpb.NNData: FP16 layer lists, and no host getTensor."""

    def __init__(self, layers):
        self._layers = layers

    def getLayerFp16(self, name):
        return list(self._layers[name])


def test_fast_script_reads_device_fp16_layers():
    script = build_fast_tracker_script(256, 144)
    calls = _nndata_method_calls(script)
    used = [call.func.attr for call in calls]
    missing = sorted({name for name in used if name not in _DEVICE_NNDATA_METHODS})
    assert missing == [], missing
    assert [
        name for name in used if name.startswith(("getLayer", "getTensor", "getFirst"))
    ] == ["getLayerFp16"]

    tensor_calls = []
    for node in ast.walk(ast.parse(script)):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "tensor_values":
            continue
        if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant):
            tensor_calls.append(node)
    tensor_calls.sort(key=lambda node: node.lineno)
    tensor_names = [node.args[1].value for node in tensor_calls]
    assert tensor_names == [name for name, _length in FAST_FP16_LAYERS]

    layers = {
        name: [float(index) for index in range(length)]
        for name, length in FAST_FP16_LAYERS
    }
    read = _tensor_values_function(script)
    for name, length in FAST_FP16_LAYERS:
        values = read(_DeviceNnData(layers), name)
        assert values == layers[name]
        assert len(values) == length


def test_setuptools_install_ships_the_device_safe_module(tmp_path):
    library = tmp_path / "site-packages"
    root = tmp_path / "root"
    camera = REPO_ROOT / "ros_packages/camera"
    try:
        subprocess.check_call(
            [
                sys.executable,
                "setup.py",
                "install",
                "--single-version-externally-managed",
                "--root",
                str(root),
                "--prefix",
                "/usr",
                "--install-lib",
                str(library),
                "--record",
                str(tmp_path / "install-record.txt"),
            ],
            cwd=camera,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        for generated in (camera / "build", camera / "oak_d_lite.egg-info"):
            if generated.exists():
                subprocess.check_call(["rm", "-rf", str(generated)])
    installed_files = list(root.rglob("hand_tracking.py"))
    assert len(installed_files) == 1
    installed = installed_files[0].read_text(encoding="utf-8")
    source = (REPO_ROOT / "ros_packages/camera/oak_d_lite/hand_tracking.py").read_text(
        encoding="utf-8"
    )
    assert installed == source
    assert "nn_data.getLayerFp16(name)" in installed
    assert "nn_data.getTensor" not in installed
    assert installed_files[0].name == "hand_tracking.py"
    assert installed_files[0].parent.name == "oak_d_lite"


def test_fast_script_letterboxes_256x144_and_stays_on_the_device():
    side, pad_w, pad_h = letterbox_square(256, 144)
    assert (side, pad_w, pad_h) == (256, 0, 56)

    script = build_fast_tracker_script(256, 144)
    compile(script, "hand_tracking_fast.py", "exec")
    assert "pad_h = 56" in script
    assert "img_w = 256" in script
    assert "img_h = 144" in script
    assert "2.9 * box_size" in script
    assert f"node.io['{FAST_PALM_CONFIG_OUTPUT}']" in script
    assert f"node.io['{FAST_LANDMARK_CONFIG_OUTPUT}']" in script
    assert f"node.io['{FAST_DECODER_INPUT}']" in script
    assert f"node.io['{FAST_LANDMARK_INPUT}']" in script
    assert f"node.io['{FAST_RESULT_OUTPUT}']" in script
    lowered = script.lower()
    assert "hostnode" not in lowered
    assert "createinputqueue" not in lowered
    assert "xlink" not in lowered


def _fast_payload(**overrides):
    crop = np.tile([112.0, 112.0, 22.4], 21).tolist()
    crop[3:5] = [224.0, 112.0]
    # The same 21 points on the letterboxed square: centre, then the right edge.
    square = np.tile([0.5, 0.5], 21).tolist()
    square[2:4] = [0.75, 0.5]
    payload = {
        "lm_score": [0.91],
        "handedness": [0.2],
        "palm_score": [0.8],
        "rotation": [0.0],
        "rect_center_x": [0.5],
        "rect_center_y": [0.5],
        "rect_size": [0.5],
        "rrn_lms": [crop],
        "sqn_lms": [square],
    }
    payload.update(overrides)
    return payload


def test_fast_result_maps_the_letterboxed_square_onto_the_published_frame():
    import marshal

    hands = parse_fast_script_result(marshal.dumps(_fast_payload()))
    assert len(hands) == 1

    detection = map_fast_hand(hands[0], 1280, 720, 256, 144)

    assert detection.keypoint_x[0] == pytest.approx(640.0)
    assert detection.keypoint_y[0] == pytest.approx(360.0)
    assert detection.keypoint_x[1] == pytest.approx(960.0)
    assert detection.keypoint_y[1] == pytest.approx(360.0)
    assert detection.keypoint_z[0] == pytest.approx(0.1)
    assert detection.landmark_score == pytest.approx(0.91)
    assert detection.palm_score == pytest.approx(0.8)
    assert detection.handedness == pytest.approx(0.2)
    assert detection.x_min <= 640 < detection.x_max
    assert detection.y_min <= 360 <= detection.y_max


def test_fast_result_drops_a_landmark_below_the_score_gate():
    import marshal

    assert parse_fast_script_result(marshal.dumps(_fast_payload(lm_score=[0.1]))) == []


def test_fast_result_ignores_the_tracking_rect_for_the_drawn_position():
    """The tracker regresses its rect every frame; that must not move the hand.

    The device sends the landmarks on the letterboxed square, so a rect that
    the single-hand tracker shifted upwards cannot pull the drawn hand with it.
    """
    import marshal

    centred = map_fast_hand(
        parse_fast_script_result(marshal.dumps(_fast_payload()))[0],
        1280,
        720,
        256,
        144,
    )
    shifted = map_fast_hand(
        parse_fast_script_result(
            marshal.dumps(_fast_payload(rect_center_y=[0.30], rect_size=[0.42]))
        )[0],
        1280,
        720,
        256,
        144,
    )

    assert shifted.keypoint_x == pytest.approx(centred.keypoint_x)
    assert shifted.keypoint_y == pytest.approx(centred.keypoint_y)


def test_fast_script_sends_the_square_landmarks():
    script = build_fast_tracker_script(256, 144)
    assert '"sqn_lms"' in script
    assert 'hand_landmarks["sqn_lms"].append(sqn_lms)' in script


def _rotate_about(icx, icy, px, py, rotation_rad):
    dx = px - icx
    dy = py - icy
    cos_r = math.cos(rotation_rad)
    sin_r = math.sin(rotation_rad)
    return icx + dx * cos_r - dy * sin_r, icy + dx * sin_r + dy * cos_r


def test_prerotated_pixel_crop_rect_matches_depthai_imagemanip_and_rr2img():
    source_width, source_height = 1152, 648
    side, pad_w, pad_h = letterbox_square(source_width, source_height)
    sqn_rr_center_x, sqn_rr_center_y, sqn_rr_size = 0.4, 0.55, 0.25
    target_cx = sqn_rr_center_x * side - pad_w
    target_cy = sqn_rr_center_y * side - pad_h
    icx = source_width * 0.5
    icy = source_height * 0.5
    expected_x = sqn_rr_center_x * source_width
    expected_y = sqn_rr_center_y * source_width - pad_h

    for angle_deg in (0.0, 30.0, 45.0, -60.0, 90.0):
        rotation = math.radians(angle_deg)
        crop_w = sqn_rr_size * side
        cx, cy, w, h, rr_angle = prerotated_pixel_crop_rect(
            target_cx,
            target_cy,
            crop_w,
            crop_w,
            rotation,
            source_width,
            source_height,
        )
        assert w == pytest.approx(crop_w)
        assert h == pytest.approx(crop_w)
        assert rr_angle == pytest.approx(angle_deg)
        landed_x, landed_y = _rotate_about(icx, icy, cx, cy, rotation)
        assert landed_x == pytest.approx(expected_x, abs=1e-6)
        assert landed_y == pytest.approx(expected_y, abs=1e-6)

    dai = pytest.importorskip("depthai")
    frame = np.zeros((source_height, source_width, 3), dtype=np.uint8)
    img = dai.ImgFrame()
    img.setData(frame.tobytes())
    img.setType(dai.ImgFrame.Type.BGR888p)
    img.setWidth(source_width)
    img.setHeight(source_height)

    lm_input = 224
    pipeline = dai.Pipeline(createImplicitDevice=False)
    manip = pipeline.create(dai.node.ImageManip)
    manip.setRunOnHost(True)
    image_queue = manip.inputImage.createInputQueue(maxSize=2, blocking=False)
    config_queue = manip.inputConfig.createInputQueue(maxSize=2, blocking=False)
    output_queue = manip.out.createOutputQueue(maxSize=2, blocking=False)
    try:
        pipeline.start()
        for angle_deg in (45.0, -60.0):
            rotation = math.radians(angle_deg)
            crop_w = sqn_rr_size * side
            cx, cy, w, h, rr_angle = prerotated_pixel_crop_rect(
                target_cx,
                target_cy,
                crop_w,
                crop_w,
                rotation,
                source_width,
                source_height,
            )
            rr = dai.RotatedRect()
            rr.center = dai.Point2f(cx, cy)
            rr.size = dai.Size2f(w, h)
            rr.angle = rr_angle
            cfg = dai.ImageManipConfig()
            cfg.addCropRotatedRect(rr, False)
            cfg.setOutputSize(
                lm_input, lm_input, dai.ImageManipConfig.ResizeMode.STRETCH
            )
            config_queue.send(cfg)
            image_queue.send(img)
            out = None
            for _ in range(100):
                out = output_queue.tryGet()
                if out is not None:
                    break
                time.sleep(0.02)
            assert out is not None
            inv = out.getTransformation().invTransformPoint
            center = inv(dai.Point2f(0.5 * lm_input, 0.5 * lm_input))
            assert center.x == pytest.approx(expected_x, abs=0.1)
            assert center.y == pytest.approx(expected_y, abs=0.1)
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)
            for rrn_x, rrn_y in ((0.25, 0.75), (0.8, 0.2)):
                mapped = inv(dai.Point2f(rrn_x * lm_input, rrn_y * lm_input))
                sqn_x = sqn_rr_center_x + sqn_rr_size * (
                    (rrn_x - 0.5) * cos_r + (0.5 - rrn_y) * sin_r
                )
                sqn_y = sqn_rr_center_y + sqn_rr_size * (
                    (rrn_y - 0.5) * cos_r + (rrn_x - 0.5) * sin_r
                )
                assert mapped.x == pytest.approx(sqn_x * side - pad_w, abs=0.1)
                assert mapped.y == pytest.approx(sqn_y * side - pad_h, abs=0.1)
    finally:
        manip.stopPipeline()
        del output_queue, config_queue, image_queue, manip, pipeline


def test_fast_script_preserves_single_hand_roi_and_averages_handedness():
    script = build_fast_tracker_script(1152, 648)
    assert "HandednessAverage" in script
    assert "nb_hands_in_previous_frame" in script
    assert "cfg_pre_pd.setBackgroundColor(0, 0, 0)" in script
    assert "cfg.addCropRotatedRect(rr, False)" in script
    assert "addCropRotatedRect(rr, True)" not in script


def test_map_fast_hand_maps_through_branch_field_window():
    import marshal

    hand = parse_fast_script_result(marshal.dumps(_fast_payload()))[0]
    _, _, pad_h = letterbox_square(1152, 648)
    side = max(1152, 648)
    sqn_y = 0.5 * side
    image_y = min(648.0, max(0.0, sqn_y - pad_h))

    mapped_default = map_fast_hand(hand, 1280, 720, 1152, 648)
    assert mapped_default.keypoint_y[0] == pytest.approx(
        image_y * 720.0 / 648.0, rel=1e-4
    )
    assert 0.0 <= mapped_default.keypoint_y[0] <= 720.0

    mapped_field = map_fast_hand(
        hand,
        1280,
        720,
        1152,
        648,
        field=(0.0, 1280.0, 36.0, 648.0),
    )
    expected_y = 36.0 + image_y * 648.0 / 648.0
    assert mapped_field.keypoint_y[0] == pytest.approx(expected_y, rel=1e-4)
    assert 36.0 <= mapped_field.keypoint_y[0] <= 684.0
