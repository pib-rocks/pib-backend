"""Pure camera deprojection helpers for the scene model (no ROS / depthai)."""

from __future__ import annotations

from typing import Mapping, MutableMapping, Optional, Sequence, Tuple

Point3d = MutableMapping[str, float]
Intrinsics = Mapping[str, float]
BBox = Sequence[float]


def bbox_center_px(bbox: BBox) -> Tuple[int, int]:
    """Return ``(u, v)`` pixel centre: u = column, v = row (same as depth lookup)."""
    x1, y1, x2, y2 = (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
    u = int((x1 + x2) / 2.0)
    v = int((y1 + y2) / 2.0)
    return u, v


def depth_mm_at_px(
    depth,
    x: int,
    y: int,
    *,
    depth_available: bool = True,
) -> Optional[float]:
    """Read cached depth in millimetres; ``None`` when unavailable or invalid."""
    if not depth_available or depth is None:
        return None
    height, width = depth.shape[:2]
    if x < 0 or y < 0 or x >= width or y >= height:
        return None
    value = float(depth[y, x])
    if value <= 0.0:
        return None
    return value


def deproject_pixel_to_camera_mm(
    u: int,
    v: int,
    depth_mm: float,
    intrinsics: Optional[Intrinsics],
) -> Optional[Point3d]:
    """Deproject one pixel to camera optical frame coordinates in millimetres."""
    if intrinsics is None:
        return None
    if depth_mm <= 0.0:
        return None
    try:
        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
    except (KeyError, TypeError, ValueError):
        return None
    if fx == 0.0 or fy == 0.0:
        return None
    z = float(depth_mm)
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy
    return {"x": x, "y": y, "z": z}


def point3d_for_detection_bbox(
    bbox: BBox,
    depth,
    intrinsics: Optional[Intrinsics],
    *,
    depth_available: bool = True,
) -> Optional[Point3d]:
    """Build ``point3d`` from bbox centre and cached depth."""
    u, v = bbox_center_px(bbox)
    depth_mm = depth_mm_at_px(depth, u, v, depth_available=depth_available)
    if depth_mm is None:
        return None
    return deproject_pixel_to_camera_mm(u, v, depth_mm, intrinsics)


def intrinsics_from_calibration_matrix(matrix) -> Optional[dict]:
    """Extract fx, fy, cx, cy from a 3x3 calibration matrix."""
    try:
        fx = float(matrix[0][0])
        fy = float(matrix[1][1])
        cx = float(matrix[0][2])
        cy = float(matrix[1][2])
    except (IndexError, TypeError, ValueError):
        return None
    if fx == 0.0 or fy == 0.0:
        return None
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
