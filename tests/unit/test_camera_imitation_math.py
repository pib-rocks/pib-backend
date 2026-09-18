"""Device-free contracts for the official two-stage imitation helpers."""

import os
import sys
import types
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from ros_packages.camera.oak_d_lite import imitation


def _rect(cx=0.5, cy=0.4, width=0.2, height=0.3, angle=12.0):
    return types.SimpleNamespace(
        center=types.SimpleNamespace(x=cx, y=cy),
        size=types.SimpleNamespace(width=width, height=height),
        angle=angle,
    )


def _detection(**kwargs):
    rect = _rect(**kwargs)
    return types.SimpleNamespace(
        confidence=0.87,
        getBoundingBox=lambda: rect,
    )


def _keypoints(x=0.25, y=0.75):
    points = [
        types.SimpleNamespace(
            imageCoordinates=types.SimpleNamespace(x=x, y=y, z=index / 10.0)
        )
        for index in range(21)
    ]
    return types.SimpleNamespace(getKeypoints=lambda: points)


def _prediction(value):
    return types.SimpleNamespace(prediction=value)


def _gathered(item, detection=None):
    return types.SimpleNamespace(
        reference_data=types.SimpleNamespace(detections=[detection or _detection()]),
        items=[item],
    )


def test_crop_config_preserves_rotation_adds_padding_and_uses_stretch():
    fake_rect = _rect()
    config = MagicMock()
    stretch = imitation.dai.ImageManipConfig.ResizeMode.STRETCH
    with (
        patch.object(imitation.dai, "RotatedRect", return_value=fake_rect),
        patch.object(
            imitation.dai, "ImageManipConfig", return_value=config
        ) as image_manip_config,
    ):
        image_manip_config.ResizeMode.STRETCH = stretch
        result = imitation.detection_crop_config(_detection(), 0.1, 224, 224)

    assert result is config
    padded = config.addCropRotatedRect.call_args.args[0]
    assert config.addCropRotatedRect.call_args.kwargs == {"normalizedCoords": True}
    assert padded.center.x == pytest.approx(0.5)
    assert padded.center.y == pytest.approx(0.4)
    assert padded.size.width == pytest.approx(0.4)
    assert padded.size.height == pytest.approx(0.5)
    assert padded.angle == pytest.approx(12.0)
    config.setOutputSize.assert_called_once_with(224, 224, stretch)
    config.setReusePreviousImage.assert_called_once_with(False)


def test_stretch_mapping_is_exact_and_clipped_to_published_frame():
    item = {
        "0": _keypoints(x=0.25, y=0.75),
        "1": _prediction(0.91),
        "2": _prediction(0.73),
    }

    hand = imitation.gathered_hands(_gathered(item), 1280, 720)[0]

    # bbox is x=[.4,.6], y=[.25,.55], then padded by exactly .1 per side.
    assert hand["landmarks"][0] == pytest.approx((0.4 * 1280, 0.525 * 720))
    assert hand["palm_score"] == pytest.approx(0.87)
    assert hand["landmark_score"] == pytest.approx(0.91)
    assert hand["handedness"] == pytest.approx(0.73)

    clipped = imitation.gathered_hands(
        _gathered(
            {"0": _keypoints(x=2.0, y=-1.0), "1": _prediction(0.9)},
            _detection(cx=0.95, cy=0.05),
        ),
        1280,
        720,
    )[0]
    assert clipped["landmarks"][0] == (1280.0, 0.0)


def test_score_threshold_world_extraction_and_missing_world_head():
    world = _keypoints(x=0.1, y=0.2)
    accepted = imitation.gathered_hands(
        _gathered(
            {
                "0": _keypoints(),
                "1": _prediction(0.5),
                "2": _prediction(0.25),
                "3": world,
            }
        ),
        1280,
        720,
    )
    rejected = imitation.gathered_hands(
        _gathered({"0": _keypoints(), "1": _prediction(0.499)}), 1280, 720
    )

    assert len(accepted[0]["world"]) == 63
    assert accepted[0]["world"][:3] == pytest.approx([0.1, 0.2, 0.0])
    assert rejected == []
    assert (
        imitation.gathered_hands(
            _gathered({"0": _keypoints(), "1": _prediction(0.9)}), 1280, 720
        )[0]["world"]
        == []
    )


def test_empty_detections_and_trace_values():
    empty = types.SimpleNamespace(
        reference_data=types.SimpleNamespace(detections=[]), items=[]
    )
    assert imitation.gathered_hands(empty, 1280, 720) == []
    assert imitation.gathered_result_trace_values(empty) == []

    trace = imitation.gathered_result_trace_values(_gathered({"1": _prediction(0.91)}))[
        0
    ]
    assert trace[0:2] == pytest.approx((0.87, 0.91))
    assert trace[2] == pytest.approx((0.3, 0.15, 0.4, 0.5))


def test_bbox_encloses_every_landmark_and_is_clamped():
    points = [(471.7, 284.4), (585.8, 472.3), (594.8, 201.2), (731.8, 348.3)]
    bbox = imitation.box_from_points(points, 1280, 720)
    assert bbox == (471, 201, 732, 473)
    assert all(bbox[0] <= x <= bbox[2] and bbox[1] <= y <= bbox[3] for x, y in points)
    assert imitation.box_from_points([], 1280, 720) == (0, 0, 1, 1)
    assert imitation.box_from_points([(-1, -2), (5000, 4000)], 1280, 720) == (
        0,
        0,
        1280,
        720,
    )


def test_world_scalar_order_is_stable():
    names, values = imitation.world_landmark_scalars(range(63))
    assert len(names) == len(values) == 63
    assert names[:4] == ["world_0_x", "world_0_y", "world_0_z", "world_1_x"]
    assert names[-1] == "world_20_z"
    assert imitation.world_landmark_scalars([]) == ([], [])
