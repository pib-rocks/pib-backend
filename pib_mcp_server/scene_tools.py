"""Format ROS scene-model snapshots for MCP read-only tools."""

from __future__ import annotations

import math
from typing import Any

_MAX_SUMMARY_CHARS = 200


def _object_age_s(stamp_ms: int, last_seen_ms: int | float | None) -> float:
    if last_seen_ms is None:
        return 0.0
    return max(0.0, (int(stamp_ms) - int(last_seen_ms)) / 1000.0)


def _distance_m(point3d: Any) -> float | None:
    if not isinstance(point3d, dict):
        return None
    z = point3d.get("z")
    if isinstance(z, (int, float)) and math.isfinite(float(z)):
        return round(float(z) / 1000.0, 3)
    return None


def _compact_object(entry: dict[str, Any], stamp_ms: int) -> dict[str, Any]:
    age_s = round(_object_age_s(stamp_ms, entry.get("last_seen_ms")), 2)
    point3d = entry.get("point3d")
    compact: dict[str, Any] = {
        "object_id": entry.get("object_id"),
        "label": entry.get("label"),
        "confidence": round(float(entry.get("confidence", 0.0)), 3),
        "distance_m": _distance_m(point3d),
        "point3d": point3d,
        "age_s": age_s,
    }
    return compact


def _compact_person(entry: dict[str, Any], stamp_ms: int) -> dict[str, Any]:
    compact = _compact_object(entry, stamp_ms)
    compact.pop("label", None)
    return compact


def _empty_scene_reason(raw_objects: list[Any]) -> str:
    if not raw_objects:
        return "no objects in the current scene snapshot"
    return "no recent detections"


def _build_summary(
    active_labels: list[str], *, detections_active: bool, reason: str
) -> str:
    if detections_active:
        labels = [label for label in active_labels if label]
        if not labels:
            text = "I see something in front of me, but labels are unavailable."
        elif len(labels) == 1:
            text = f"I see a {labels[0]}."
        elif len(labels) == 2:
            text = f"I see a {labels[0]} and a {labels[1]}."
        else:
            head = ", ".join(labels[:-1])
            text = f"I see a {head}, and a {labels[-1]}."
    else:
        text = "I do not see any objects right now."
    if len(text) > _MAX_SUMMARY_CHARS:
        return text[: _MAX_SUMMARY_CHARS - 3].rstrip() + "..."
    return text


def build_describe_scene_response(scene: dict[str, Any]) -> dict[str, Any]:
    stamp_ms = int(scene.get("stamp_ms", 0))
    raw_objects = [
        item for item in (scene.get("objects") or []) if isinstance(item, dict)
    ]
    active = [item for item in raw_objects if not item.get("stale")]
    non_people = [
        item for item in active if str(item.get("label", "")).casefold() != "person"
    ]
    objects = [_compact_object(item, stamp_ms) for item in non_people]
    raw_people = [
        item for item in (scene.get("people") or []) if isinstance(item, dict)
    ]
    active_people = [item for item in raw_people if not item.get("stale")]
    people = [_compact_person(item, stamp_ms) for item in active_people]
    detections_active = bool(objects or people)
    reason = "" if detections_active else _empty_scene_reason(raw_objects)
    labels = [str(item.get("label", "")) for item in active]
    summary = _build_summary(
        labels,
        detections_active=detections_active,
        reason=reason,
    )
    return {
        "ok": True,
        "summary": summary,
        "detections_active": detections_active,
        "reason": reason,
        "objects": objects,
        "people": people,
    }


def build_locate_object_response(scene: dict[str, Any], name: str) -> dict[str, Any]:
    needle = (name or "").strip().casefold()
    if not needle:
        return {"ok": False, "reason": "not_found"}
    stamp_ms = int(scene.get("stamp_ms", 0))
    matches: list[tuple[dict[str, Any], float, bool]] = []
    for entry in scene.get("objects") or []:
        if not isinstance(entry, dict):
            continue
        label = str(entry.get("label", ""))
        if label.casefold() != needle:
            continue
        age_s = _object_age_s(stamp_ms, entry.get("last_seen_ms"))
        matches.append((entry, age_s, bool(entry.get("stale"))))
    if not matches:
        return {"ok": False, "reason": "not_found"}

    matches.sort(
        key=lambda row: (
            -float(row[0].get("confidence", 0.0)),
            row[1],
        )
    )
    best, age_s, stale = matches[0]
    if stale:
        return {"ok": False, "reason": "stale", "age_s": round(age_s, 2)}
    point3d = best.get("point3d")
    return {
        "ok": True,
        "object_id": best.get("object_id"),
        "label": best.get("label"),
        "confidence": round(float(best.get("confidence", 0.0)), 3),
        "distance_m": _distance_m(point3d),
        "point3d": point3d,
        "age_s": round(age_s, 2),
    }
