import json

import numpy as np

from ros_packages.camera.oak_d_lite.scene_model import SceneModel, fixture_detections
from ros_packages.camera.oak_d_lite.scene_model_projection import (
    bbox_center_px,
    deproject_pixel_to_camera_mm,
    depth_mm_at_px,
    point3d_for_detection_bbox,
)


def _ingest(model: SceneModel, detections, stamp_ms, width=1280, height=720):
    model.ingest(detections, width, height, stamp_ms)


def test_identity_stable_across_ten_second_occlusion_for_ten_object_fixture():
    model = SceneModel(ttl_s=30.0, match_iou=0.3)
    fixture = fixture_detections(10, prefix="thing")
    t0 = 1_000_000
    _ingest(model, fixture, t0)
    before = {o["object_id"] for o in model.snapshot(t0)["objects"]}
    assert len(before) == 10

    t_after_gap = t0 + 10_000
    snap_gap = model.snapshot(t_after_gap)
    assert len(snap_gap["objects"]) == 10
    assert all(entry["stale"] is False for entry in snap_gap["objects"])

    ids_before = {
        entry["label"]: entry["object_id"] for entry in model.snapshot(t0)["objects"]
    }
    _ingest(model, fixture, t_after_gap)
    ids_after = {
        entry["label"]: entry["object_id"]
        for entry in model.snapshot(t_after_gap)["objects"]
    }
    assert len(ids_after) == 10
    stable = sum(1 for label in ids_before if ids_before[label] == ids_after.get(label))
    assert stable >= 8


def test_new_object_gets_new_id():
    model = SceneModel(match_iou=0.3)
    t = 0
    first = {"label": "cup", "confidence": 0.8, "bbox": (10, 10, 50, 50)}
    _ingest(model, [first], t)
    id_a = model.snapshot(t)["objects"][0]["object_id"]

    second = {"label": "cup", "confidence": 0.85, "bbox": (200, 200, 240, 240)}
    _ingest(model, [first, second], t + 100)
    snap = model.snapshot(t + 100)
    by_bbox = {tuple(o["bbox"]): o["object_id"] for o in snap["objects"]}
    assert len(by_bbox) == 2
    assert by_bbox[(10, 10, 50, 50)] == id_a
    assert by_bbox[(200, 200, 240, 240)] != id_a


def test_moved_object_beyond_iou_does_not_steal_existing_id():
    model = SceneModel(match_iou=0.3)
    t = 0
    anchor = {"label": "bottle", "confidence": 0.9, "bbox": (100, 100, 160, 180)}
    _ingest(model, [anchor], t)
    original_id = model.snapshot(t)["objects"][0]["object_id"]

    moved = {"label": "bottle", "confidence": 0.92, "bbox": (400, 100, 460, 180)}
    _ingest(model, [anchor, moved], t + 50)
    snap = model.snapshot(t + 50)
    by_bbox = {tuple(o["bbox"]): o["object_id"] for o in snap["objects"]}
    assert by_bbox[(100, 100, 160, 180)] == original_id
    assert by_bbox[(400, 100, 460, 180)] != original_id


def test_stale_marking_after_ttl():
    model = SceneModel(ttl_s=30.0)
    det = {"label": "book", "confidence": 0.7, "bbox": (0, 0, 40, 40)}
    t0 = 0
    _ingest(model, [det], t0)
    fresh = model.snapshot(t0 + 29_999)["objects"][0]
    assert fresh["stale"] is False
    stale = model.snapshot(t0 + 30_000)["objects"][0]
    assert stale["stale"] is True
    assert stale["last_seen_ms"] == t0


def test_snapshot_json_serialisable_with_required_fields():
    model = SceneModel()
    person = {"label": "person", "confidence": 0.99, "bbox": (10, 10, 100, 200)}
    other = {
        "label": "chair",
        "confidence": 0.5,
        "bbox": (300, 300, 380, 380),
        "point3d": {"x": 1.0, "y": 2.0, "z": 500.0},
    }
    stamp = 42
    _ingest(model, [person, other], stamp)
    snap = model.snapshot(stamp)
    encoded = json.dumps(snap)
    decoded = json.loads(encoded)
    assert decoded["stamp_ms"] == stamp
    required = {
        "object_id",
        "label",
        "confidence",
        "point3d",
        "bbox",
        "last_seen_ms",
        "stale",
    }
    for obj in decoded["objects"]:
        assert required <= set(obj.keys())
    assert len(decoded["people"]) == 1
    assert decoded["people"][0]["label"] == "person"


def test_deprojection_reproduces_known_point_within_one_mm():
    intrinsics = {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 400.0}
    expected = {"x": 400.0, "y": 200.0, "z": 2000.0}
    u = 700
    v = 500
    got = deproject_pixel_to_camera_mm(u, v, expected["z"], intrinsics)
    assert got is not None
    assert abs(got["x"] - expected["x"]) <= 1.0
    assert abs(got["y"] - expected["y"]) <= 1.0
    assert abs(got["z"] - expected["z"]) <= 1.0


def test_deprojection_none_for_zero_depth_out_of_frame_and_missing_intrinsics():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 10.0, "cy": 10.0}
    depth = np.array([[0, 500], [800, 1200]], dtype=np.uint16)
    assert deproject_pixel_to_camera_mm(1, 1, 0.0, intrinsics) is None
    assert depth_mm_at_px(depth, 0, 0) is None
    assert depth_mm_at_px(depth, 5, 1) is None
    assert deproject_pixel_to_camera_mm(1, 1, 500.0, None) is None
    assert point3d_for_detection_bbox((0, 0, 2, 2), depth, intrinsics) is not None
    assert point3d_for_detection_bbox((0, 0, 2, 2), depth, intrinsics) == {
        "x": -21.6,
        "y": -21.6,
        "z": 1200.0,
    }


def test_bbox_centre_uses_column_row_depth_index_order():
    depth = np.zeros((120, 200), dtype=np.uint16)
    u, v = 150, 70
    depth[v, u] = 888
    bbox = (140, 60, 160, 80)
    assert bbox_center_px(bbox) == (u, v)
    assert depth_mm_at_px(depth, u, v) == 888.0
    assert depth_mm_at_px(depth, v, u) is None
    intrinsics = {"fx": 100.0, "fy": 100.0, "cx": 0.0, "cy": 0.0}
    point = point3d_for_detection_bbox(bbox, depth, intrinsics)
    assert point is not None
    assert point["z"] == 888.0
    assert point["x"] == 888.0 * u / 100.0
    assert point["y"] == 888.0 * v / 100.0


def test_snapshot_includes_intrinsics_metadata():
    model = SceneModel()
    model.set_intrinsics_metadata(
        {"fx": 1.0, "fy": 2.0, "cx": 3.0, "cy": 4.0},
        None,
    )
    snap = model.snapshot(0)
    assert snap["intrinsics"] == {"fx": 1.0, "fy": 2.0, "cx": 3.0, "cy": 4.0}
    assert "intrinsics_unavailable_reason" not in snap

    model.set_intrinsics_metadata(None, "pipeline_not_running")
    missing = model.snapshot(1)
    assert missing["intrinsics"] is None
    assert missing["intrinsics_unavailable_reason"] == "pipeline_not_running"


def test_init_scene_model_loads_intrinsics_when_pipeline_already_started(monkeypatch):
    """The camera starts its pipeline before the scene model exists (PR-2224)."""
    import sys
    import types

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {"data": ""})
    std_msgs.msg = std_msgs_msg
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg)

    from ros_packages.camera.oak_d_lite.scene_model_mixin import SceneModelMixin

    srv = types.ModuleType("std_srvs.srv")
    srv.Trigger = object
    pkg = types.ModuleType("std_srvs")
    pkg.srv = srv
    monkeypatch.setitem(sys.modules, "std_srvs", pkg)
    monkeypatch.setitem(sys.modules, "std_srvs.srv", srv)

    class _Node(SceneModelMixin):
        def __init__(self):
            self.pipeline = object()
            self.loaded = 0

        def create_publisher(self, *args, **kwargs):
            return None

        def create_timer(self, *args, **kwargs):
            return None

        def create_service(self, *args, **kwargs):
            return None

        def _load_scene_model_intrinsics_once(self):
            self.loaded += 1

    node = _Node()
    node._init_scene_model()
    assert node.loaded == 1
