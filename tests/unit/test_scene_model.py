import json

from ros_packages.camera.oak_d_lite.scene_model import SceneModel, fixture_detections


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
    stable = sum(
        1 for label in ids_before if ids_before[label] == ids_after.get(label)
    )
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
