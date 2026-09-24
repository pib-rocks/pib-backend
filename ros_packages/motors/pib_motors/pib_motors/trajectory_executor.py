"""Fixed-rate software trajectory controller for JointTrajectory messages.

Interpolates each joint independently over ``time_from_start`` via-points using
monotone cubic Hermite (Fritsch–Carlson / PCHIP) so motion blends through
via-points instead of stopping. Bricklet motion config is sharpened for the
duration of the trajectory and restored afterwards, following the backup /
modify / restore-in-finally pattern used by ``StartupPoseExecutor``.

This module is intentionally free of ROS runtime imports so unit tests can
exercise interpolation and the executor with plain Python fakes.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Iterable, Mapping, Protocol, Sequence

DEFAULT_RATE_HZ = 100.0
# High bricklet motion limits so the software spline, not the servo ramp, shapes speed.
DEFAULT_SHARP_VELOCITY = 50000
DEFAULT_SHARP_ACCELERATION = 50000
DEFAULT_SHARP_DECELERATION = 50000


class MotorLike(Protocol):
    name: str

    def set_position(self, position: int) -> bool: ...

    def get_position(self) -> int: ...

    def get_settings(self) -> dict[str, Any]: ...

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool: ...


class LoggerLike(Protocol):
    def info(self, msg: str) -> Any: ...

    def warn(self, msg: str) -> Any: ...

    def error(self, msg: str) -> Any: ...


class NodeLike(Protocol):
    def get_logger(self) -> LoggerLike: ...


MotorLookup = Mapping[str, Sequence[MotorLike]]


def duration_to_seconds(time_from_start: Any) -> float:
    """Convert a ROS Duration, duck-typed duration, or numeric seconds to float."""
    if time_from_start is None:
        return 0.0
    if isinstance(time_from_start, (int, float)):
        return float(time_from_start)
    if hasattr(time_from_start, "to_sec"):
        return float(time_from_start.to_sec())
    sec = getattr(time_from_start, "sec", 0) or 0
    nanosec = getattr(time_from_start, "nanosec", None)
    if nanosec is None:
        nanosec = getattr(time_from_start, "nanosecs", 0) or 0
    return float(sec) + float(nanosec) * 1e-9


def is_software_trajectory(jt: Any) -> bool:
    """True when ``jt`` is a timed / waypoint trajectory, not a legacy zip pose.

    Existing pib callers (poses, Blockly, MCP ``apply_pose``) send one
    JointTrajectoryPoint per motor, each with a single position and no
    ``time_from_start``. That shape must keep using ``as_motor_positions``.

    A software trajectory is:
    - any message with a non-zero ``time_from_start``, or
    - more than one waypoint in the standard JointTrajectory layout
      (each point carries a position per joint).
    """
    points = list(getattr(jt, "points", []) or [])
    if not points:
        return False
    if any(
        duration_to_seconds(getattr(p, "time_from_start", None)) > 0.0 for p in points
    ):
        return True
    names = list(getattr(jt, "joint_names", []) or [])
    n_joints = len(names)
    if len(points) <= 1 or n_joints == 0:
        return False
    # Legacy zip: N names, N points, each point has a single position.
    if len(points) == n_joints and all(
        len(getattr(p, "positions", []) or []) <= 1 for p in points
    ):
        return False
    return all(len(getattr(p, "positions", []) or []) >= n_joints for p in points)


def monotone_cubic_slopes(
    times: Sequence[float], values: Sequence[float]
) -> list[float]:
    """Fritsch–Carlson slopes for a monotone cubic Hermite spline."""
    n = len(times)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    h = [times[i + 1] - times[i] for i in range(n - 1)]
    delta = [
        (values[i + 1] - values[i]) / h[i] if h[i] != 0.0 else 0.0 for i in range(n - 1)
    ]
    m = [0.0] * n
    m[0] = delta[0]
    m[-1] = delta[-1]
    for i in range(1, n - 1):
        if delta[i - 1] * delta[i] <= 0.0:
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            denom = (w1 / delta[i - 1]) + (w2 / delta[i])
            m[i] = (w1 + w2) / denom if denom != 0.0 else 0.0
    return m


def interpolate_joint(
    times: Sequence[float], values: Sequence[float], t: float
) -> float:
    """Monotone cubic sample of one joint at time ``t`` (clamped to the spline ends)."""
    if not times or not values:
        return 0.0
    if len(times) == 1 or t <= times[0]:
        return float(values[0])
    if t >= times[-1]:
        return float(values[-1])

    slopes = monotone_cubic_slopes(times, values)
    i = 0
    for idx in range(len(times) - 1):
        if times[idx] <= t <= times[idx + 1]:
            i = idx
            break
    h = times[i + 1] - times[i]
    if h == 0.0:
        return float(values[i])
    s = (t - times[i]) / h
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return (
        h00 * values[i]
        + h10 * h * slopes[i]
        + h01 * values[i + 1]
        + h11 * h * slopes[i + 1]
    )


def interpolate_positions(
    joint_names: Sequence[str],
    times: Sequence[float],
    positions_by_joint: Mapping[str, Sequence[float]],
    t: float,
) -> dict[str, int]:
    """Return rounded integer targets for every joint at time ``t``."""
    return {
        name: int(round(interpolate_joint(times, positions_by_joint[name], t)))
        for name in joint_names
    }


def parse_joint_trajectory(
    jt: Any,
) -> tuple[list[str], list[float], dict[str, list[float]]]:
    """Unpack ``joint_names`` + waypoint ``positions`` / ``time_from_start``."""
    names = list(getattr(jt, "joint_names", []) or [])
    series: dict[str, list[float]] = {name: [] for name in names}
    timed: list[tuple[float, Any]] = []
    for point in getattr(jt, "points", []) or []:
        timed.append(
            (duration_to_seconds(getattr(point, "time_from_start", None)), point)
        )
    timed.sort(key=lambda item: item[0])
    times: list[float] = []
    for t, point in timed:
        times.append(t)
        positions = list(getattr(point, "positions", []) or [])
        for i, name in enumerate(names):
            if i < len(positions):
                series[name].append(float(positions[i]))
            elif series[name]:
                series[name].append(series[name][-1])
            else:
                series[name].append(0.0)
    return names, times, series


class TrajectoryExecutor:
    """Run a JointTrajectory by streaming interpolated ``set_position`` commands."""

    def __init__(
        self,
        node: NodeLike | None,
        motor_lookup: MotorLookup,
        rate_hz: float = DEFAULT_RATE_HZ,
        motion_sharp: dict[str, int] | None = None,
        restore: bool = True,
        sleep_fn: Callable[[float], None] = time.sleep,
        clock_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self.node = node
        self.motor_lookup = motor_lookup
        self.rate_hz = rate_hz if rate_hz > 0 else DEFAULT_RATE_HZ
        self.motion_sharp = motion_sharp or {
            "velocity": DEFAULT_SHARP_VELOCITY,
            "acceleration": DEFAULT_SHARP_ACCELERATION,
            "deceleration": DEFAULT_SHARP_DECELERATION,
        }
        self.restore = restore
        self.sleep_fn = sleep_fn
        self.clock_fn = clock_fn

    def execute(self, jt: Any) -> bool:
        """Execute ``jt``. Multi-point timed paths interpolate; one-point is a single set."""
        names = list(getattr(jt, "joint_names", []) or [])
        points = list(getattr(jt, "points", []) or [])
        if not names or not points:
            self._log_error("joint trajectory is missing joint_names or points")
            return False

        motors = self._resolve_motors(names)
        if motors is None:
            return False

        if len(points) <= 1 or not is_software_trajectory(jt):
            return self._execute_single_point(names, points)

        names, times, series = parse_joint_trajectory(jt)
        original_settings = self._backup_motor_settings(motors)
        self._sharpen_motion(motors, original_settings)
        try:
            return self._run_interpolation_loop(names, times, series)
        finally:
            if self.restore:
                self._restore_settings(motors, original_settings)

    def _execute_single_point(
        self, names: Sequence[str], points: Sequence[Any]
    ) -> bool:
        """Backward-compatible one-shot ``set_position`` (no motion-config change)."""
        success = True
        for name, point in zip(names, points):
            positions = list(getattr(point, "positions", []) or [])
            if not positions:
                success = False
                continue
            position = int(round(float(positions[0])))
            for motor in self.motor_lookup[name]:
                success &= bool(motor.set_position(position))
        return success

    def _run_interpolation_loop(
        self,
        names: Sequence[str],
        times: Sequence[float],
        series: Mapping[str, Sequence[float]],
    ) -> bool:
        dt = 1.0 / self.rate_hz
        duration = times[-1] if times else 0.0
        t0 = self.clock_fn()
        success = True
        while True:
            now = self.clock_fn() - t0
            targets = interpolate_positions(names, times, series, now)
            for name in names:
                position = targets[name]
                for motor in self.motor_lookup[name]:
                    success &= bool(motor.set_position(position))
            if now >= duration:
                break
            self.sleep_fn(dt)
        return success

    def _resolve_motors(self, names: Iterable[str]) -> list[MotorLike] | None:
        resolved: list[MotorLike] = []
        seen: set[int] = set()
        for name in names:
            if name not in self.motor_lookup:
                self._log_error(f"unknown joint name '{name}'")
                return None
            for motor in self.motor_lookup[name]:
                identity = id(motor)
                if identity not in seen:
                    seen.add(identity)
                    resolved.append(motor)
        return resolved

    def _backup_motor_settings(
        self, motors: Sequence[MotorLike]
    ) -> dict[str, dict[str, Any]]:
        original_settings: dict[str, dict[str, Any]] = {}
        for motor in motors:
            settings = motor.get_settings()
            if settings:
                original_settings[motor.name] = settings.copy()
        return original_settings

    def _sharpen_motion(
        self,
        motors: Sequence[MotorLike],
        original_settings: Mapping[str, dict[str, Any]],
    ) -> None:
        for motor in motors:
            if motor.name not in original_settings:
                continue
            sharp = original_settings[motor.name].copy()
            sharp["velocity"] = self.motion_sharp["velocity"]
            sharp["acceleration"] = self.motion_sharp["acceleration"]
            sharp["deceleration"] = self.motion_sharp["deceleration"]
            motor.apply_settings(sharp)

    def _restore_settings(
        self,
        motors: Sequence[MotorLike],
        original_settings: Mapping[str, dict[str, Any]],
    ) -> None:
        for motor in motors:
            if motor.name in original_settings:
                motor.apply_settings(original_settings[motor.name])

    def _log_error(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().error(message)
