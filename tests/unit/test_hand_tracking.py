"""Device-free tests for hand decoder and landmark coordinate handling."""

import math
from pathlib import Path

import numpy as np
import pytest

from ros_packages.camera.oak_d_lite.hand_tracking import (
    PalmRegion,
    decode_palm_result,
    fit_manip_crop,
    map_landmarks_to_frame,
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


def test_decoder_maps_top10_record_and_bbox_to_full_frame():
    tensor = np.zeros((10, 8), dtype=np.float32)
    tensor[0] = [0.9, 0.5, 0.5, 0.2, 0.5, 0.6, 0.5, 0.4]

    palms = decode_palm_result(tensor)

    assert len(palms) == 1
    assert palms[0].score == pytest.approx(0.9)
    assert palms[0].rotation == pytest.approx(0.0)
    assert palms[0].bbox_pixels(1000, 500) == (400, 150, 600, 350)


def test_bbox_maps_from_letterboxed_nn_branch_to_preview_frame():
    palm = PalmRegion(0.9, 0.5, 0.25, 0.0, 0.5, 0.25, 0.2, 0.0)

    # The 1280x720 NN branch is letterboxed into a 1280x1280 square. A point
    # at normalized y=0.25 therefore lies 1/18 down the unpadded source image.
    assert palm.bbox_pixels(
        frame_width=640,
        frame_height=480,
        source_width=1280,
        source_height=720,
    ) == (320, 27, 320, 27)


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

    assert points[0] == pytest.approx((500.0, 450.0))


def test_landmarks_empty_tensor_returns_empty():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.4, 0.0)
    assert map_landmarks_to_frame([], palm, 100, 100) == []


def test_landmarks_reject_wrong_shape():
    palm = PalmRegion(0.9, 0.5, 0.5, 0.2, 0.5, 0.5, 0.4, 0.0)
    with pytest.raises(ValueError, match="exactly 63"):
        map_landmarks_to_frame(np.zeros(62), palm, 100, 100)


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
