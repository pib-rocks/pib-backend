"""Pure, deterministic conversion of named hand landmarks into palm poses.

Image X/Y values are pixels. Non-zero landmark Z values are millimetres. When
depth is unavailable (the camera contract uses zero), ``fallback_depth_mm`` is
used and image distances are scaled by the configured physical palm width.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


HAND_LANDMARK_NAMES = (
    "wrist",
    "thumb_cmc",
    "thumb_mcp",
    "thumb_ip",
    "thumb_tip",
    "index_finger_mcp",
    "index_finger_pip",
    "index_finger_dip",
    "index_finger_tip",
    "middle_finger_mcp",
    "middle_finger_pip",
    "middle_finger_dip",
    "middle_finger_tip",
    "ring_finger_mcp",
    "ring_finger_pip",
    "ring_finger_dip",
    "ring_finger_tip",
    "pinky_mcp",
    "pinky_pip",
    "pinky_dip",
    "pinky_tip",
)


@dataclass(frozen=True)
class Point3:
    """A point in millimetres."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PalmPose:
    """Palm centre and orientation in the optical camera frame."""

    position: Point3
    quaternion_xyzw: tuple[float, float, float, float]
    scale_mm_per_pixel: float
    used_depth_fallback: bool


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _subtract(a: Point3, b: Point3) -> Point3:
    return Point3(a.x - b.x, a.y - b.y, a.z - b.z)


def _cross(a: Point3, b: Point3) -> Point3:
    return Point3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def _norm(vector: Point3) -> float:
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


def _normalise(vector: Point3, label: str) -> Point3:
    magnitude = _norm(vector)
    if not math.isfinite(magnitude) or magnitude <= 1e-9:
        raise ValueError(f"cannot derive palm {label} axis from coincident landmarks")
    return Point3(vector.x / magnitude, vector.y / magnitude, vector.z / magnitude)


def _rotation_to_quaternion(
    x_axis: Point3, y_axis: Point3, z_axis: Point3
) -> tuple[float, float, float, float]:
    """Convert a column-major orthonormal rotation matrix to XYZW."""
    m00, m01, m02 = x_axis.x, y_axis.x, z_axis.x
    m10, m11, m12 = x_axis.y, y_axis.y, z_axis.y
    m20, m21, m22 = x_axis.z, y_axis.z, z_axis.z
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quaternion = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    length = math.sqrt(sum(component * component for component in quaternion))
    return tuple(component / length for component in quaternion)


def _quaternion_multiply(
    first: Sequence[float], second: Sequence[float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def parse_named_landmarks(
    names: Sequence[str],
    x_values: Sequence[float],
    y_values: Sequence[float],
    z_values: Sequence[float],
    *,
    frame_width: int,
    frame_height: int,
    fallback_depth_mm: float = 500.0,
    palm_width_mm: float = 80.0,
) -> tuple[dict[str, Point3], float, bool]:
    """Validate/reorder the 21 parallel arrays and return optical-frame points.

    A finite non-zero Z is retained per landmark. Zero Z values use the median
    valid depth in the same hand, or ``fallback_depth_mm`` when all are invalid.
    """
    expected_count = len(HAND_LANDMARK_NAMES)
    lengths = (len(names), len(x_values), len(y_values), len(z_values))
    if lengths != (expected_count,) * 4:
        raise ValueError("hand detection must contain four parallel 21-value arrays")
    if len(set(names)) != expected_count or set(names) != set(HAND_LANDMARK_NAMES):
        raise ValueError("hand detection must contain each expected landmark name once")
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")
    if not _finite([*x_values, *y_values]):
        raise ValueError("landmark X/Y values must be finite")
    if not math.isfinite(fallback_depth_mm) or fallback_depth_mm <= 0.0:
        raise ValueError("fallback depth must be finite and positive")
    if not math.isfinite(palm_width_mm) or palm_width_mm <= 0.0:
        raise ValueError("palm width must be finite and positive")

    by_name = {
        name: (float(x_values[index]), float(y_values[index]), float(z_values[index]))
        for index, name in enumerate(names)
    }
    valid_depths = sorted(
        value[2]
        for value in by_name.values()
        if math.isfinite(value[2]) and value[2] > 0.0
    )
    if valid_depths:
        middle = len(valid_depths) // 2
        depth = (
            valid_depths[middle]
            if len(valid_depths) % 2
            else (valid_depths[middle - 1] + valid_depths[middle]) / 2.0
        )
    else:
        depth = float(fallback_depth_mm)

    index_xy = by_name["index_finger_mcp"]
    pinky_xy = by_name["pinky_mcp"]
    palm_width_pixels = math.hypot(index_xy[0] - pinky_xy[0], index_xy[1] - pinky_xy[1])
    if palm_width_pixels <= 1e-9:
        raise ValueError("index and pinky MCP landmarks must not coincide")
    scale = palm_width_mm / palm_width_pixels

    points = {}
    used_fallback = False
    for name in HAND_LANDMARK_NAMES:
        x_pixel, y_pixel, z_mm = by_name[name]
        if not math.isfinite(z_mm) or z_mm <= 0.0:
            z_mm = depth
            used_fallback = True
        points[name] = Point3(
            (x_pixel - frame_width / 2.0) * scale,
            (y_pixel - frame_height / 2.0) * scale,
            z_mm,
        )
    return points, scale, used_fallback


def palm_pose_from_landmarks(
    names: Sequence[str],
    x_values: Sequence[float],
    y_values: Sequence[float],
    z_values: Sequence[float],
    *,
    frame_width: int,
    frame_height: int,
    fallback_depth_mm: float = 500.0,
    palm_width_mm: float = 80.0,
) -> PalmPose:
    """Derive palm centre, orthonormal axes, scale, and depth provenance."""
    points, scale, used_fallback = parse_named_landmarks(
        names,
        x_values,
        y_values,
        z_values,
        frame_width=frame_width,
        frame_height=frame_height,
        fallback_depth_mm=fallback_depth_mm,
        palm_width_mm=palm_width_mm,
    )
    centre_names = ("wrist", "index_finger_mcp", "middle_finger_mcp", "pinky_mcp")
    centre = Point3(
        sum(points[name].x for name in centre_names) / len(centre_names),
        sum(points[name].y for name in centre_names) / len(centre_names),
        sum(points[name].z for name in centre_names) / len(centre_names),
    )
    x_axis = _normalise(
        _subtract(points["index_finger_mcp"], points["pinky_mcp"]), "X"
    )
    palm_up = _subtract(points["middle_finger_mcp"], points["wrist"])
    z_axis = _normalise(_cross(x_axis, palm_up), "normal")
    y_axis = _normalise(_cross(z_axis, x_axis), "Y")
    return PalmPose(
        position=centre,
        quaternion_xyzw=_rotation_to_quaternion(x_axis, y_axis, z_axis),
        scale_mm_per_pixel=scale,
        used_depth_fallback=used_fallback,
    )


def map_to_robot_target(
    hand_pose: PalmPose,
    *,
    origin_mm: Sequence[float],
    reference_depth_mm: float,
    position_gain: float = 1.0,
) -> PalmPose:
    """Map optical right/down/forward axes to robot forward/left/up axes."""
    if len(origin_mm) != 3 or not _finite(origin_mm):
        raise ValueError("target origin must contain three finite values")
    if not _finite((reference_depth_mm, position_gain)):
        raise ValueError("target mapping parameters must be finite")
    hand = hand_pose.position
    target = Point3(
        float(origin_mm[0]) + (reference_depth_mm - hand.z) * position_gain,
        float(origin_mm[1]) - hand.x * position_gain,
        float(origin_mm[2]) - hand.y * position_gain,
    )
    # Proper optical-frame rotation maps camera forward/right/down to robot
    # forward/right/down (+X/-Y/-Z). Translation remains the mirrored gesture
    # mapping documented above; this rotation keeps the target quaternion valid.
    optical_to_robot = (0.5, -0.5, 0.5, -0.5)
    return PalmPose(
        target,
        _quaternion_multiply(optical_to_robot, hand_pose.quaternion_xyzw),
        hand_pose.scale_mm_per_pixel,
        hand_pose.used_depth_fallback,
    )
