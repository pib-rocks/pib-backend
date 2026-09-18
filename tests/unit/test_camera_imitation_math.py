"""Device-free contract tests for the on-device imitation manager."""

import math
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from ros_packages.camera.oak_d_lite import imitation
from ros_packages.camera.oak_d_lite.imitation import (
    build_imitation_script,
    fit_landmark_region,
    landmark_pixels_to_square,
    landmark_score_passes,
    normalize_radians,
    palm_regions,
    square_box_to_frame,
    square_points_to_frame,
    world_landmark_scalars,
)


def _record(score=0.9, size=0.2):
    return [score, 0.5, 0.5, size, 0.4, 0.5, 0.6, 0.5]


def test_palm_threshold_invalid_layout_and_empty_top_two():
    assert palm_regions(_record(0.499) + _record(0.0)) == []
    assert palm_regions(_record(0.5) + _record(0.0, 0.0))[0]["palm_score"] == 0.5
    assert palm_regions([0.0] * 15) == []
    assert palm_regions(_record(float("nan")) + _record(0.0)) == []


def test_reference_rotation_center_and_2_9_roi_scale_are_exact():
    region = palm_regions(_record() + _record(0.0))[0]

    assert normalize_radians(region["rotation"]) == pytest.approx(math.pi / 2)
    assert region["center_x"] == pytest.approx(0.6)
    assert region["center_y"] == pytest.approx(0.5)
    assert region["size"] == pytest.approx(0.58)


def test_landmark_threshold_bites_at_point_five():
    assert landmark_score_passes(0.499) is False
    assert landmark_score_passes(0.5) is True
    assert landmark_score_passes(float("nan")) is False


def test_rotated_landmarks_map_from_crop_to_square():
    region = {
        "center_x": 0.5,
        "center_y": 0.5,
        "size": 0.4,
        "rotation": math.pi / 2,
    }
    values = [112.0, 0.0, 0.0] * 21

    points = landmark_pixels_to_square(values, region)

    assert len(points) == 21
    assert points[0] == pytest.approx((0.7, 0.5))


def test_landmark_region_near_an_edge_keeps_its_size():
    """A hand near the edge must keep the ROI scale the landmark net expects.

    Shrinking the ROI to force it inside the image changes how large the hand
    appears in the 224x224 crop, and that net is scale sensitive: measured on
    the robot, an off-centre hand then produced no detection at all while a
    centred one scored 0.997. The reference lets the rect overhang and relies on
    border replication, so a valid region must come back untouched.
    """
    region = {
        "center_x": 0.1,
        "center_y": 0.3,
        "size": 0.58,
        "rotation": math.pi / 4,
    }

    fitted = fit_landmark_region(region, 256, 144)

    assert fitted["size"] == region["size"]
    assert fitted["rotation"] == region["rotation"]
    assert fitted["center_x"] == region["center_x"]


def test_landmark_region_is_rejected_when_its_centre_leaves_the_image():
    """Border replication cannot recover a centre outside the frame."""
    outside = {
        "center_x": 1.4,
        "center_y": 0.3,
        "size": 0.2,
        "rotation": 0.0,
    }
    degenerate = {
        "center_x": 0.5,
        "center_y": 0.3,
        "size": 0.0,
        "rotation": 0.0,
    }

    assert fit_landmark_region(outside, 256, 144) is None
    assert fit_landmark_region(degenerate, 256, 144) is None


def test_landscape_square_padding_maps_back_to_frame_pixels():
    points = square_points_to_frame([(0.5, 0.5), (0.25, 0.359375)], 1280, 720)

    assert points == [(640.0, 360.0), (320.0, 180.0)]


def test_bbox_is_non_degenerate_and_clamped():
    region = {"box_x": 0.0, "box_y": 0.21875, "box_size": 0.001}

    x_min, y_min, x_max, y_max = square_box_to_frame(region, 1280, 720)

    assert (x_min, y_min) == (0, 0)
    assert x_max > x_min
    assert y_max > y_min


def test_world_layout_has_21_xyz_parallel_scalars():
    names, values = world_landmark_scalars([float(index) for index in range(63)])

    assert len(names) == len(values) == 63
    assert names[:4] == ["world_0_x", "world_0_y", "world_0_z", "world_1_x"]
    assert names[-1] == "world_20_z"
    assert world_landmark_scalars([]) == ([], [])


def test_script_embeds_tested_source_and_v3_api_boundaries():
    script = build_imitation_script()

    compile(script, "<imitation-script>", "exec")
    assert "def palm_regions(" in script
    assert "read_layer(packet, name)" in script
    assert 'read_layer(packet, "result")' in script
    assert "getLayerFp16" in script
    assert "ResizeMode.LETTERBOX" in script
    # Every setOutputSize call in the embedded script must pass a resize mode:
    # the Script runtime has no two-argument overload and dies at runtime.
    for call in re.findall(r"setOutputSize\(([^)]*)\)", script):
        assert "ResizeMode." in call, call
    assert "ResizeMode.STRETCH" in script
    assert "addCropRotatedRect" in script and "setCropRotatedRect" in script
    # v3 has no border replication; an overhanging ROI must be filled, not dropped.
    assert "setBackgroundColor" in script
    assert "setWarpBorderReplicatePixels(" not in script
    assert "output = Buffer(len(data))" in script
    assert 'node.io["host"].send(output)' in script


def test_box_from_points_encloses_every_landmark():
    """Decision (a): the published box must contain all 21 keypoints."""
    points = [
        (471.7, 284.4),
        (585.8, 472.3),
        (594.8, 201.2),
        (661.8, 221.0),
        (731.8, 348.3),
    ]
    x_min, y_min, x_max, y_max = imitation.box_from_points(points, 1280, 720)

    for x, y in points:
        assert x_min <= x <= x_max, (x, x_min, x_max)
        assert y_min <= y <= y_max, (y, y_min, y_max)
    assert (x_min, y_min, x_max, y_max) == (471, 201, 732, 473)


def test_box_from_points_beats_the_palm_box_on_real_measured_values():
    """The palm box left the measured fingertips outside; this must not recur."""
    # Live values measured on the robot, hand open in front of the camera.
    region = {"box_x": 0.5754, "box_y": 0.2089, "box_size": 0.0977}
    landmarks = [
        (471.7, 284.4),
        (585.8, 472.3),
        (594.8, 201.2),
        (661.8, 221.0),
        (731.8, 348.3),
    ]
    palm = imitation.square_box_to_frame(region, 1280, 720)
    derived = imitation.box_from_points(landmarks, 1280, 720)

    outside_palm = [
        point
        for point in landmarks
        if not (palm[0] <= point[0] <= palm[2] and palm[1] <= point[1] <= palm[3])
    ]
    assert outside_palm, "test data must reproduce the palm-box problem"
    assert all(
        derived[0] <= point[0] <= derived[2] and derived[1] <= point[1] <= derived[3]
        for point in landmarks
    )


def test_box_from_points_stays_inside_the_frame_and_non_degenerate():
    x_min, y_min, x_max, y_max = imitation.box_from_points(
        [(-50.0, -20.0), (5000.0, 4000.0)], 1280, 720
    )

    assert (x_min, y_min) == (0, 0)
    assert (x_max, y_max) == (1280, 720)
    assert x_max > x_min and y_max > y_min


def test_box_from_points_handles_an_empty_point_list():
    assert imitation.box_from_points([], 1280, 720) == (0, 0, 1, 1)


def test_imitation_source_keeps_the_published_frame_aspect_ratio():
    """The square-normalised coordinates are mapped back with the published
    frame's dimensions, so the branch feeding the nets must share its aspect
    ratio. A non-16:9 source silently shifts every keypoint.
    """
    from ros_packages.camera.oak_d_lite import stereo

    width = stereo.IMITATION_SOURCE_WIDTH
    height = stereo.IMITATION_SOURCE_HEIGHT

    assert width * 9 == height * 16, (width, height)


def test_imitation_source_is_large_enough_for_the_landmark_crop():
    """The landmark net wants 224x224 real pixels. A palm region is roughly a
    fifth of the frame width, so anything below ~1120 px upscales mush.
    """
    from ros_packages.camera.oak_d_lite import stereo

    palm_region_px = stereo.IMITATION_SOURCE_WIDTH / 5.0

    assert palm_region_px >= 224, palm_region_px
