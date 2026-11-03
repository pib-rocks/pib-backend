"""Persistent scene state from detection dicts (no ROS dependencies)."""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence, Tuple

BBox = Tuple[float, float, float, float]
Point3d = Dict[str, float]

_PEOPLE_LABELS = frozenset({"person"})


def _bbox_iou(a: BBox, b: BBox) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    inter_w = max(0.0, ix2 - ix1)
    inter_h = max(0.0, iy2 - iy1)
    inter = inter_w * inter_h
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def _normalise_bbox(raw: Sequence[float]) -> BBox:
    if len(raw) != 4:
        raise ValueError("bbox must have four coordinates")
    x1, y1, x2, y2 = (float(v) for v in raw)
    return (x1, y1, x2, y2)


def _normalise_point3d(raw: Any) -> Optional[Point3d]:
    if raw is None:
        return None
    if isinstance(raw, Mapping):
        if "x" in raw and "y" in raw and "z" in raw:
            return {
                "x": float(raw["x"]),
                "y": float(raw["y"]),
                "z": float(raw["z"]),
            }
    raise ValueError("point3d must be null or a mapping with x, y, z")


class _Track:
    __slots__ = (
        "object_id",
        "label",
        "confidence",
        "bbox",
        "point3d",
        "last_seen_ms",
    )

    def __init__(
        self,
        object_id: int,
        label: str,
        confidence: float,
        bbox: BBox,
        point3d: Optional[Point3d],
        last_seen_ms: int,
    ) -> None:
        self.object_id = object_id
        self.label = label
        self.confidence = confidence
        self.bbox = bbox
        self.point3d = point3d
        self.last_seen_ms = last_seen_ms


class SceneModel:
    """Track detections with stable ids, occlusion tolerance, and staleness."""

    def __init__(self, ttl_s: float = 30.0, match_iou: float = 0.3) -> None:
        self.ttl_s = float(ttl_s)
        self.ttl_ms = self.ttl_s * 1000.0
        self.match_iou = float(match_iou)
        self._tracks: MutableMapping[int, _Track] = {}
        self._next_id = 1

    def ingest(
        self,
        detections: Sequence[Mapping[str, Any]],
        frame_width: int,
        frame_height: int,
        stamp_ms: int,
    ) -> None:
        """Update tracks from one frame of detection dicts."""
        del frame_width, frame_height  # reserved for future normalisation
        stamp_ms = int(stamp_ms)
        parsed: List[Tuple[str, float, BBox, Optional[Point3d]]] = []
        for item in detections:
            label = str(item.get("label", ""))
            confidence = float(item.get("confidence", 0.0))
            bbox = _normalise_bbox(item["bbox"])
            point3d = _normalise_point3d(item.get("point3d"))
            parsed.append((label, confidence, bbox, point3d))

        parsed.sort(key=lambda row: row[1], reverse=True)
        available = set(self._tracks.keys())

        for label, confidence, bbox, point3d in parsed:
            best_id: Optional[int] = None
            best_iou = 0.0
            for track_id in available:
                track = self._tracks[track_id]
                if track.label != label:
                    continue
                iou = _bbox_iou(bbox, track.bbox)
                if iou >= self.match_iou and iou > best_iou:
                    best_iou = iou
                    best_id = track_id
            if best_id is None:
                track_id = self._next_id
                self._next_id += 1
                self._tracks[track_id] = _Track(
                    track_id, label, confidence, bbox, point3d, stamp_ms
                )
                continue
            track = self._tracks[best_id]
            track.confidence = confidence
            track.bbox = bbox
            track.point3d = point3d
            track.last_seen_ms = stamp_ms
            available.discard(best_id)

    def snapshot(self, stamp_ms: int) -> Dict[str, Any]:
        """Return a JSON-serialisable view of all known objects."""
        stamp_ms = int(stamp_ms)
        objects: List[Dict[str, Any]] = []
        people: List[Dict[str, Any]] = []
        for track_id in sorted(self._tracks.keys()):
            track = self._tracks[track_id]
            age_ms = float(stamp_ms - track.last_seen_ms)
            stale = age_ms >= self.ttl_ms
            entry = {
                "object_id": track.object_id,
                "label": track.label,
                "confidence": float(track.confidence),
                "point3d": track.point3d,
                "bbox": [
                    float(track.bbox[0]),
                    float(track.bbox[1]),
                    float(track.bbox[2]),
                    float(track.bbox[3]),
                ],
                "last_seen_ms": int(track.last_seen_ms),
                "stale": bool(stale),
            }
            objects.append(entry)
            if track.label.lower() in _PEOPLE_LABELS:
                people.append(dict(entry))
        return {
            "stamp_ms": stamp_ms,
            "objects": objects,
            "people": people,
        }


def fixture_detections(count: int, prefix: str = "obj") -> List[Dict[str, Any]]:
    """Build non-overlapping boxes for tests (grid layout)."""
    cols = max(1, int(count**0.5 + 0.999))
    cell_w, cell_h = 80, 60
    margin = 10
    out: List[Dict[str, Any]] = []
    for index in range(count):
        row = index // cols
        col = index % cols
        x1 = margin + col * (cell_w + margin)
        y1 = margin + row * (cell_h + margin)
        out.append(
            {
                "label": f"{prefix}_{index}",
                "confidence": 0.9,
                "bbox": (x1, y1, x1 + cell_w, y1 + cell_h),
            }
        )
    return out
