"""Unit tests for the device-side hand manager Script.

The Script runs on the OAK, so pytest cannot execute it.  What is testable - and
what this file pins - is the geometry it embeds, the call shapes the device
firmware accepts, and the two device-runtime traps that cost a deployment each:

* ``setOutputSize`` needs its ``ResizeMode`` on the device; the two-argument form
  raises ``TypeError`` inside the Script and kills the pipeline.
* the crop has to be fitted in BRANCH PIXELS per axis.  Treating the normalized
  decoder coordinates as a letterboxed square places the rect off the image, the
  ImageManip then drops the frame, and the Script waits forever for a landmark
  result - which trips the device watchdog.
"""

import math

import pytest

from ros_packages.camera.oak_d_lite.hand_device import (
    PALM_RECORD_COUNT,
    build_hand_script,
    fit_landmark_region,
    landmark_pixels_to_square,
    palm_regions,
)


def _record(score, box_x, box_y, box_size, kp0, kp2):
    return [score, box_x, box_y, box_size, kp0[0], kp0[1], kp2[0], kp2[1]]


def _tensor(records):
    values = [0.0] * (8 * PALM_RECORD_COUNT)
    for index, record in enumerate(records):
        values[index * 8 : index * 8 + 8] = record
    return values


def test_script_carries_the_device_call_shapes():
    script = build_hand_script(2104, 1560)

    # The device runtime keeps the 2.x-era reader; getTensor belongs to the host.
    assert "getLayerFp16" in script
    assert ".getTensor(" not in script
    # Both crop configs are produced on the device.
    assert "pre_pd_manip_cfg" in script
    assert "pre_lm_manip_cfg" in script
    assert "addCropRotatedRect" in script
    # The zoo head returns TOP 10 records.
    assert f"PALM_RECORD_COUNT = {PALM_RECORD_COUNT}" in script


def test_every_set_output_size_carries_a_resize_mode():
    script = build_hand_script(2104, 1560)

    calls = [
        line.strip()
        for line in script.split("\n")
        if "setOutputSize(" in line and not line.strip().startswith("#")
    ]
    assert calls, "the script must size its crops"
    for call in calls:
        assert "ResizeMode." in call, call


def test_palm_regions_gate_and_order_the_ten_records():
    tensor = _tensor(
        [
            _record(0.62, 0.5, 0.4, 0.2, (0.5, 0.6), (0.5, 0.4)),
            _record(0.31, 0.2, 0.2, 0.1, (0.2, 0.3), (0.2, 0.1)),
            _record(0.94, 0.3, 0.3, 0.25, (0.3, 0.4), (0.3, 0.2)),
        ]
    )

    regions = palm_regions(tensor)

    # 0.31 is below the gate; the rest is ordered best first.
    assert [round(region["palm_score"], 2) for region in regions] == [0.94, 0.62]
    assert regions[0]["size"] == pytest.approx(2.9 * 0.25)


def test_fit_landmark_region_keeps_the_crop_inside_the_branch():
    region = {
        "palm_score": 0.9,
        "box_x": 0.05,
        "box_y": 0.5,
        "box_size": 0.2,
        "center_x": 0.05,
        "center_y": 0.5,
        "size": 0.58,
        "rotation": 0.0,
    }

    fitted = fit_landmark_region(region, 2104, 1560)

    assert fitted is not None
    half = fitted["size"] / 2.0
    assert fitted["center_x"] - half >= 0.0
    assert fitted["center_x"] + half <= 1.0
    assert fitted["center_y"] - half >= 0.0
    assert fitted["center_y"] + half <= 1.0
    # A palm at the edge keeps its centre and shrinks instead of moving off it.
    assert fitted["center_x"] == pytest.approx(0.05, abs=0.01)
    assert fitted["size"] < region["size"]


def test_fit_landmark_region_rejects_an_impossible_region():
    region = {
        "center_x": 0.0,
        "center_y": 0.0,
        "size": 0.5,
        "rotation": 0.0,
        "palm_score": 0.9,
    }

    assert fit_landmark_region(region, 0, 1560) is None


def test_landmark_mapping_rotates_in_pixels_not_in_normalized_space():
    region = {
        "center_x": 0.5,
        "center_y": 0.5,
        "size": 0.2,
        "rotation": math.pi / 2.0,
        "palm_score": 0.9,
    }
    values = [0.0] * 63
    # A point one quarter input to the right of the crop centre.
    values[0], values[1], values[2] = 224.0 * 0.75, 112.0, -8.0

    points = landmark_pixels_to_square(values, region, 2104, 1560)

    assert len(points) == 21
    x, y, z = points[0]
    # Rotated by 90 degrees in pixel space, then normalized per axis - the branch
    # aspect (2104x1560) must survive in the result.
    assert x != y
    assert z == pytest.approx(-8.0 / 224.0)
