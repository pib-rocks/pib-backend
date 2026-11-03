"""Unit tests for MCP scene describe/locate tools."""

from __future__ import annotations

import asyncio

from pib_mcp_server.backend import BackendError
from pib_mcp_server.scene_tools import (
    build_describe_scene_response,
    build_locate_object_response,
)
from pib_mcp_server.server import create_server


class FakeSceneBackend:
    def __init__(self, scene: dict | None = None, *, error: BackendError | None = None):
        self.scene = scene
        self.error = error
        self.calls: list[tuple[str, ...]] = []

    def get_scene(self):
        self.calls.append(("get_scene",))
        if self.error is not None:
            raise self.error
        assert self.scene is not None
        return self.scene


def _call_tool(server, name: str, arguments: dict | None = None):
    arguments = arguments or {}
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    if hasattr(result, "structured_output") and result.structured_output is not None:
        return result.structured_output
    if hasattr(result, "content") and result.content:
        import json

        c0 = result.content[0]
        if hasattr(c0, "text"):
            try:
                return json.loads(c0.text)
            except Exception:
                pass
    return result


def test_describe_scene_parses_active_objects_and_people():
    scene = {
        "stamp_ms": 10_000,
        "objects": [
            {
                "object_id": 1,
                "label": "cup",
                "confidence": 0.91,
                "point3d": {"x": 10.0, "y": 20.0, "z": 1500.0},
                "last_seen_ms": 9_500,
                "stale": False,
            },
            {
                "object_id": 2,
                "label": "person",
                "confidence": 0.88,
                "point3d": None,
                "last_seen_ms": 9_900,
                "stale": False,
            },
        ],
        "people": [
            {
                "object_id": 2,
                "label": "person",
                "confidence": 0.88,
                "point3d": None,
                "last_seen_ms": 9_900,
                "stale": False,
            }
        ],
    }
    result = build_describe_scene_response(scene)
    assert result["ok"] is True
    assert result["detections_active"] is True
    assert result["reason"] == ""
    assert len(result["objects"]) == 1
    assert result["objects"][0]["label"] == "cup"
    assert result["objects"][0]["distance_m"] == 1.5
    assert result["objects"][0]["age_s"] == 0.5
    assert len(result["people"]) == 1
    assert "person" not in result["summary"].lower() or "I see" in result["summary"]


def test_describe_scene_empty_snapshot_is_honest():
    scene = {"stamp_ms": 0, "objects": [], "people": []}
    result = build_describe_scene_response(scene)
    assert result["ok"] is True
    assert result["detections_active"] is False
    assert result["objects"] == []
    assert result["people"] == []
    assert result["reason"] == "no objects in the current scene snapshot"
    assert "do not see any objects" in result["summary"]


def test_describe_scene_only_stale_tracks_are_not_an_empty_room():
    scene = {
        "stamp_ms": 40_000,
        "objects": [
            {
                "object_id": 3,
                "label": "book",
                "confidence": 0.6,
                "point3d": None,
                "last_seen_ms": 0,
                "stale": True,
            }
        ],
        "people": [],
    }
    result = build_describe_scene_response(scene)
    assert result["detections_active"] is False
    assert result["objects"] == []
    assert result["reason"] == "no recent detections"


def test_locate_object_case_insensitive_match():
    scene = {
        "stamp_ms": 1_000,
        "objects": [
            {
                "object_id": 1,
                "label": "Cup",
                "confidence": 0.5,
                "point3d": {"x": 0.0, "y": 0.0, "z": 2000.0},
                "last_seen_ms": 900,
                "stale": False,
            },
            {
                "object_id": 2,
                "label": "cup",
                "confidence": 0.9,
                "point3d": {"x": 1.0, "y": 2.0, "z": 1000.0},
                "last_seen_ms": 950,
                "stale": False,
            },
        ],
    }
    result = build_locate_object_response(scene, "CUP")
    assert result["ok"] is True
    assert result["object_id"] == 2
    assert result["confidence"] == 0.9
    assert result["distance_m"] == 1.0


def test_locate_object_refuses_stale_match():
    scene = {
        "stamp_ms": 50_000,
        "objects": [
            {
                "object_id": 4,
                "label": "phone",
                "confidence": 0.95,
                "point3d": None,
                "last_seen_ms": 0,
                "stale": True,
            }
        ],
    }
    result = build_locate_object_response(scene, "phone")
    assert result == {"ok": False, "reason": "stale", "age_s": 50.0}


def test_locate_object_not_found():
    scene = {"stamp_ms": 0, "objects": []}
    assert build_locate_object_response(scene, "lamp") == {
        "ok": False,
        "reason": "not_found",
    }


def test_describe_scene_surfaces_missing_service_error():
    backend = FakeSceneBackend(
        error=BackendError("ros_service_failed", "Service /get_scene does not exist")
    )
    server = create_server(backend)
    payload = _call_tool(server, "pib_describe_scene")
    assert payload["ok"] is False
    assert payload["detections_active"] is False
    assert payload["reason"] == "ros_service_failed"
    assert payload["objects"] == []
    assert backend.calls == [("get_scene",)]
