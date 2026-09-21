import sys
import types

from ros_packages.camera.oak_d_lite.parsed_detections import (
    translate_detection,
    translate_detections,
)


def _point(x, y, z=0.0, name=""):
    return types.SimpleNamespace(
        imageCoordinates=types.SimpleNamespace(x=x, y=y, z=z),
        labelName=name,
    )


def test_translates_label_score_box_keypoints_and_scalars_to_ros_contract(monkeypatch):
    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")

    class Detection:
        pass

    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    box = types.SimpleNamespace(
        center=types.SimpleNamespace(x=0.5, y=0.5),
        size=types.SimpleNamespace(width=0.5, height=0.25),
    )
    parsed = types.SimpleNamespace(
        label=0,
        labelName="",
        confidence=0.875,
        getBoundingBox=lambda: box,
        getKeypoints=lambda: [
            _point(0.25, 0.5, name="left_eye"),
            _point(0.75, 0.25),
        ],
        scalar_names=["yaw_deg"],
        scalar_values=[12.5],
    )

    detection = translate_detection(parsed, ("Face",), 640, 480)

    assert detection.label == "Face"
    assert detection.score == 0.875
    assert (
        detection.x_min,
        detection.y_min,
        detection.x_max,
        detection.y_max,
    ) == (160, 180, 480, 300)
    assert detection.keypoint_names == ["left_eye", "landmark_1"]
    assert detection.keypoint_x == [160.0, 480.0]
    assert detection.keypoint_y == [240.0, 120.0]
    assert detection.keypoint_z == [0.0, 0.0]
    assert detection.scalar_names == ["yaw_deg"]
    assert detection.scalar_values == [12.5]


def test_translates_two_coco_classes_to_pixel_boxes_without_keypoints(monkeypatch):
    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")

    class Detection:
        pass

    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    person = types.SimpleNamespace(
        label=0,
        labelName="",
        confidence=0.91,
        getBoundingBox=lambda: types.SimpleNamespace(
            center=types.SimpleNamespace(x=0.25, y=0.25),
            size=types.SimpleNamespace(width=0.5, height=0.5),
        ),
        scalar_names=(),
        scalar_values=(),
    )
    bicycle = types.SimpleNamespace(
        label=1,
        labelName="",
        confidence=0.42,
        getBoundingBox=lambda: types.SimpleNamespace(
            center=types.SimpleNamespace(x=0.75, y=0.75),
            size=types.SimpleNamespace(width=0.5, height=0.5),
        ),
        scalar_names=(),
        scalar_values=(),
    )

    detections = translate_detections(
        types.SimpleNamespace(detections=[person, bicycle]),
        ("person", "bicycle"),
        640,
        640,
    )

    assert [item.label for item in detections] == ["person", "bicycle"]
    assert [item.score for item in detections] == [0.91, 0.42]
    assert (
        detections[0].x_min,
        detections[0].y_min,
        detections[0].x_max,
        detections[0].y_max,
    ) == (0, 0, 320, 320)
    assert (
        detections[1].x_min,
        detections[1].y_min,
        detections[1].x_max,
        detections[1].y_max,
    ) == (320, 320, 640, 640)
    assert detections[0].keypoint_names == []
    assert detections[1].keypoint_names == []
