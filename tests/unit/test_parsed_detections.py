import sys
import types

from ros_packages.camera.oak_d_lite.parsed_detections import translate_detection


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
