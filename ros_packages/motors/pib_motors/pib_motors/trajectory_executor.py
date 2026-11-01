"""Fixed-rate software trajectory controller for JointTrajectory messages.

Interpolates each joint independently over ``time_from_start`` via-points using
monotone cubic Hermite (Fritsch–Carlson / PCHIP) so motion blends through
via-points instead of stopping. Servo speed and acceleration are coupled to the
path for the duration of the trajectory and restored afterwards, following the
backup / modify / restore-in-finally pattern used by ``StartupPoseExecutor``.

This module is intentionally free of ROS runtime imports so unit tests can
exercise interpolation and the executor with plain Python fakes.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass
from queue import SimpleQueue
from typing import Any, Callable, Iterable, Mapping, Protocol, Sequence

from pib_motors.bus_writer import BusPriority
from pib_motors.servo_speed_curve import (
    SERVO_FAMILY_FINGER,
    SLOW_PATH_BELOW_FLOOR_POLICY,
    SLOW_PATH_POLICY_REJECT,
    STS_SPEED_MAX,
    STS_SPEED_MIN,
    continuous_path_floor_centideg_per_sec,
    curve_for_family,
    family_for_servo_id,
    rad_per_sec_from_centideg_per_sec,
    speed_for_path,
)
from pib_motors.perf_trace import (
    format_perf_trace_trajectory_leg_motion,
    format_perf_trace_trajectory_slow_path,
    log_perf_trace,
)

DEFAULT_RATE_HZ = 100.0
# Upper bound for timed pose sequences (sum of leg seconds / final time_from_start).
MAX_SOFTWARE_TRAJECTORY_DURATION_SEC = 300.0

# The STS speed setting is not a linear rate: it is sub-linear and then saturates, measured at
# 2092 / 4729 / 4949 centideg per second at the settings 200 / 700 / 2000 on an STS3095, and
# 1741 / 3483 / 3835 on a finger servo. A single scalar factor was therefore wrong by a factor of
# four depending on the operating point. The curve per servo family and the measurement live in
# pib_motors/servo_speed_curve.py.
PATH_SPEED_TRACKING_MARGIN = 1.2
STS_VELOCITY_MIN = STS_SPEED_MIN
STS_VELOCITY_MAX = STS_SPEED_MAX
# Finite-difference Δt for PCHIP path speed (seconds); documented approximation at 100 Hz.
PATH_VELOCITY_FINITE_DIFF_SEC = 0.005


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
    """True when ``jt`` is a multi-point timed trajectory, not a single-point command.

    One trajectory point (with one position per joint) is an immediate target command.
    More than one point — each carrying a full position vector — is executed by software
    interpolation over ``time_from_start``.
    """
    points = list(getattr(jt, "points", []) or [])
    if len(points) <= 1:
        return False
    names = list(getattr(jt, "joint_names", []) or [])
    n_joints = len(names)
    if n_joints == 0:
        return False
    if any(
        duration_to_seconds(getattr(p, "time_from_start", None)) > 0.0 for p in points
    ):
        return True
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


def interpolate_joint_velocity_centideg_per_sec(
    times: Sequence[float],
    values: Sequence[float],
    t: float,
    dt: float = PATH_VELOCITY_FINITE_DIFF_SEC,
) -> float:
    """Path speed in centidegrees per second at ``t`` (PCHIP, forward finite difference)."""
    if not times or not values or len(times) == 1:
        return 0.0
    t_lo = times[0]
    t_hi = times[-1]
    t_sample = min(max(t, t_lo), t_hi)
    span = t_hi - t_lo
    if span <= 0.0:
        return 0.0
    dt_eff = min(max(dt, 1e-9), span)
    t1 = min(t_sample + dt_eff, t_hi)
    if t1 <= t_sample:
        return 0.0
    p0 = interpolate_joint(times, values, t_sample)
    p1 = interpolate_joint(times, values, t1)
    return (p1 - p0) / (t1 - t_sample)


def active_trajectory_leg_index(times: Sequence[float], t: float) -> int:
    """Index ``i`` of the leg ``[times[i], times[i+1]]`` containing ``t``."""
    if len(times) < 2:
        return 0
    if t >= times[-1]:
        return len(times) - 2
    for index in range(len(times) - 1):
        if t < times[index + 1]:
            return index
    return len(times) - 2


def derive_sts_velocity_setting(
    path_centideg_per_sec: float,
    *,
    curve: Sequence[tuple[int, float]] | None = None,
    margin: float = PATH_SPEED_TRACKING_MARGIN,
    min_speed: int = STS_VELOCITY_MIN,
    max_speed: int = STS_VELOCITY_MAX,
) -> int:
    """Convert a path rate into the servo speed setting that tracks it."""
    # An empty family name fell through to the high-speed table without the measured low-speed
    # points, so the default path silently ignored that measurement. Name the fallback family.
    active = tuple(curve) if curve else curve_for_family(SERVO_FAMILY_FINGER)
    return speed_for_path(
        path_centideg_per_sec,
        active,
        margin=margin,
        min_speed=min_speed,
        max_speed=max_speed,
    )


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


def validate_software_trajectory_schedule(jt: Any) -> str | None:
    """Return a human-readable error, or ``None`` when the schedule is executable."""
    if not is_software_trajectory(jt):
        return None
    _, times, _ = parse_joint_trajectory(jt)
    if len(times) < 2:
        return "software trajectory requires at least two waypoints"
    for index in range(1, len(times)):
        leg = times[index] - times[index - 1]
        if leg <= 0.0:
            if times[index] == times[index - 1]:
                return (
                    f"duplicate waypoint time_from_start {times[index]} s "
                    "is not allowed"
                )
            return (
                f"zero-second leg between {times[index - 1]} s and "
                f"{times[index]} s is not allowed"
            )
    total = times[-1]
    if total > MAX_SOFTWARE_TRAJECTORY_DURATION_SEC:
        return (
            f"trajectory duration {total} s exceeds the maximum "
            f"{MAX_SOFTWARE_TRAJECTORY_DURATION_SEC} s"
        )
    return None


def minimum_path_speed_centideg_per_sec(
    times: Sequence[float],
    values: Sequence[float],
    *,
    sample_step_sec: float = 0.05,
) -> float:
    """Smallest absolute path speed along a spline (centideg/s), sampled uniformly in time."""
    if len(times) < 2 or len(values) < 2:
        return 0.0
    duration = times[-1] - times[0]
    if duration <= 0.0:
        return 0.0
    step = max(1e-6, float(sample_step_sec))
    minimum = float("inf")
    t = times[0]
    while t <= times[-1] + 1e-9:
        speed = abs(interpolate_joint_velocity_centideg_per_sec(times, values, t))
        if speed > 0.0:
            minimum = min(minimum, speed)
        t += step
    return 0.0 if minimum == float("inf") else minimum


def family_for_joint_name(joint_name: str, motor_lookup: MotorLookup) -> str:
    """Map a trajectory joint to a servo family using actuator ids when available."""
    for motor in motor_lookup.get(joint_name, []):
        for actuator in getattr(motor, "actuators", []):
            pin = getattr(actuator, "pin", None)
            if pin is not None:
                return family_for_servo_id(int(pin))
    return SERVO_FAMILY_FINGER


def slow_path_policy_rejects() -> bool:
    """True when the slow-path policy is set to reject.

    Read through the module instead of the imported constant: the constant is bound at import
    time, so setting it later (a config change, a test) had no effect - the reject branch was
    dead code while it looked configurable.
    """
    curve_module = sys.modules.get("pib_motors.servo_speed_curve")
    if curve_module is None:
        return SLOW_PATH_BELOW_FLOOR_POLICY == SLOW_PATH_POLICY_REJECT
    return (
        getattr(curve_module, "SLOW_PATH_BELOW_FLOOR_POLICY", None)
        == SLOW_PATH_POLICY_REJECT
    )


def assess_slow_path_trajectory(
    joint_names: Sequence[str],
    times: Sequence[float],
    series: Mapping[str, Sequence[float]],
    family_for_joint: Callable[[str], str],
) -> tuple[str | None, list[tuple[str, float, float]]]:
    """Return ``(reject_reason, warnings)`` for paths below the continuous floor.

    Each warning entry is ``(joint_name, min_path_cdps, floor_cdps)``. When the floor is still
    ``None`` (measurement pending), this is a no-op.
    """
    warnings: list[tuple[str, float, float]] = []
    for name in joint_names:
        values = series.get(name)
        if not values:
            continue
        family = family_for_joint(name)
        floor = continuous_path_floor_centideg_per_sec(family)
        if floor is None or floor <= 0.0:
            continue
        min_path = minimum_path_speed_centideg_per_sec(times, values)
        if min_path <= 0.0 or min_path >= floor:
            continue
        if slow_path_policy_rejects():
            return (
                (
                    f"joint '{name}' path speed {min_path:.2f} centideg/s is below the "
                    f"continuous floor {floor:.2f} centideg/s for {family} servos"
                ),
                [],
            )
        warnings.append((name, min_path, floor))
    return None, warnings


@dataclass(frozen=True)
class TrajectoryCompletion:
    """Result handed to ``motor_control`` after a background run finishes."""

    successful: bool
    cancelled: bool
    joint_names: tuple[str, ...]
    duration_sec: float = 0.0


class TrajectoryExecutor:
    """Run a JointTrajectory by streaming interpolated ``set_position`` commands."""

    def __init__(
        self,
        node: NodeLike | None,
        motor_lookup: MotorLookup,
        rate_hz: float = DEFAULT_RATE_HZ,
        sleep_fn: Callable[[float], None] = time.sleep,
        clock_fn: Callable[[], float] = time.monotonic,
        path_speed_margin: float = PATH_SPEED_TRACKING_MARGIN,
        path_coupled_acceleration: bool = True,
        acceleration_scale: float = 2000.0,
        log_every_second: bool = True,
    ) -> None:
        self.node = node
        self.motor_lookup = motor_lookup
        self.rate_hz = rate_hz if rate_hz > 0 else DEFAULT_RATE_HZ
        self.sleep_fn = sleep_fn
        self.clock_fn = clock_fn
        self.path_speed_margin = path_speed_margin
        self.path_coupled_acceleration = path_coupled_acceleration
        self.acceleration_scale = (
            acceleration_scale if acceleration_scale > 0 else 2000.0
        )
        self.log_every_second = log_every_second
        self._last_setting: dict[str, int] = {}
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._worker_lock = threading.Lock()

    def is_running(self) -> bool:
        with self._worker_lock:
            worker = self._worker
            return worker is not None and worker.is_alive()

    def cancel(self, timeout: float | None = 60.0) -> bool:
        """Stop a background run and wait for motion-config restore."""
        self._stop_event.set()
        with self._worker_lock:
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout=timeout)
        return not worker.is_alive()

    def join(self, timeout: float | None = None) -> bool:
        """Wait until the background worker exits (tests and measurement scripts)."""
        with self._worker_lock:
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout=timeout)
        return not worker.is_alive()

    def start_async(
        self, jt: Any, completion_queue: SimpleQueue[TrajectoryCompletion] | None = None
    ) -> tuple[bool, str]:
        """Run a software trajectory on a worker thread; returns immediately when accepted."""
        if not is_software_trajectory(jt):
            return False, "not a software trajectory"
        schedule_error = validate_software_trajectory_schedule(jt)
        if schedule_error is not None:
            return False, schedule_error
        if slow_path_policy_rejects():
            names, times, series = parse_joint_trajectory(jt)
            reject, _ = assess_slow_path_trajectory(
                names,
                times,
                series,
                lambda n: family_for_joint_name(n, self.motor_lookup),
            )
            if reject is not None:
                return False, reject
        with self._worker_lock:
            if self._worker is not None and self._worker.is_alive():
                return False, "trajectory worker is still running"

            self._stop_event.clear()

            def _worker_main() -> None:
                completion = self._execute_software_trajectory(jt)
                if completion_queue is not None:
                    completion_queue.put(completion)
                with self._worker_lock:
                    self._worker = None

            self._worker = threading.Thread(
                target=_worker_main, name="trajectory_executor", daemon=True
            )
            self._worker.start()
        return True, ""

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

        self._stop_event.clear()
        completion = self._execute_software_trajectory(jt)
        return completion.successful and not completion.cancelled

    def _execute_software_trajectory(self, jt: Any) -> TrajectoryCompletion:
        names = list(getattr(jt, "joint_names", []) or [])
        schedule_error = validate_software_trajectory_schedule(jt)
        if schedule_error is not None:
            self._log_error(schedule_error)
            return TrajectoryCompletion(False, False, tuple(names), 0.0)

        motors = self._resolve_motors(names)
        if motors is None:
            return TrajectoryCompletion(False, False, tuple(names), 0.0)

        names, times, series = parse_joint_trajectory(jt)
        cancelled = False
        success = True
        run_started = self.clock_fn()
        reject, slow_warnings = assess_slow_path_trajectory(
            names, times, series, lambda n: family_for_joint_name(n, self.motor_lookup)
        )
        if reject is not None:
            self._log_error(reject)
            return TrajectoryCompletion(False, False, tuple(names), 0.0)
        for joint, min_path, floor in slow_warnings:
            self._log_perf_trace(
                format_perf_trace_trajectory_slow_path(
                    joint, min_path, floor, rad_per_sec_from_centideg_per_sec(floor)
                )
            )
        # No motion configuration is written for the duration of a trajectory any more: the
        # speed travels with each position write. That removes the whole backup/restore dance
        # and with it the class of bug where a cancel left a motor sharpened.
        success, cancelled = self._run_interpolation_loop(names, times, series, motors)
        duration_sec = max(0.0, self.clock_fn() - run_started)
        return TrajectoryCompletion(
            successful=success and not cancelled,
            cancelled=cancelled,
            joint_names=tuple(names),
            duration_sec=duration_sec,
        )

    def _execute_single_point(
        self, names: Sequence[str], points: Sequence[Any]
    ) -> bool:
        """One-shot ``set_position`` for a single trajectory point (no motion-config change)."""
        if not points:
            return False
        point = points[0]
        positions = list(getattr(point, "positions", []) or [])
        if len(positions) < len(names):
            self._log_error(
                f"trajectory point has {len(positions)} positions for "
                f"{len(names)} joint names"
            )
            return False
        success = True
        for index, name in enumerate(names):
            position = int(round(float(positions[index])))
            for motor in self.motor_lookup[name]:
                success &= bool(motor.set_position(position))
        return success

    def _run_interpolation_loop(
        self,
        names: Sequence[str],
        times: Sequence[float],
        series: Mapping[str, Sequence[float]],
        motors: Sequence[MotorLike],
    ) -> tuple[bool, bool]:
        dt = 1.0 / self.rate_hz
        duration = times[-1] if times else 0.0
        t0 = self.clock_fn()
        success = True
        cancelled = False
        last_log = -1.0
        while True:
            if self._stop_event.is_set():
                cancelled = True
                break
            now = self.clock_fn() - t0
            targets = interpolate_positions(names, times, series, now)
            for name in names:
                position = targets[name]
                # Per tick, not per leg: the slope of a PCHIP segment changes inside the
                # segment, so a speed fixed for the whole segment runs ahead of or behind
                # the path at the segment ends.
                path_speed = interpolate_joint_velocity_centideg_per_sec(
                    times, series[name], now
                )
                for motor in self.motor_lookup[name]:
                    setting = derive_sts_velocity_setting(
                        path_speed,
                        curve=self._speed_curve_for(motor),
                        margin=self.path_speed_margin,
                    )
                    acceleration = self._acceleration_for(motor, setting, dt)
                    success &= self._stream_set_position(
                        motor, position, setting, acceleration
                    )
            if self.log_every_second and now - last_log >= 1.0:
                last_log = now
                self._log_tick_speeds(names, times, series, now)
            if now >= duration:
                break
            self.sleep_fn(dt)
        return success, cancelled

    def _acceleration_for(
        self, motor: MotorLike, setting: int, dt: float
    ) -> int | None:
        """Acceleration for this write, coupled to how fast the required speed changes.

        The servo's own acceleration stays the upper bound: the point is to let it follow the
        path, not to make it snappier than it was configured to be.
        """
        if not self.path_coupled_acceleration:
            return None
        previous = self._last_setting.get(motor.name)
        self._last_setting[motor.name] = setting
        if previous is None or dt <= 0.0:
            return None
        change_per_sec = abs(setting - previous) / dt
        baseline = self._baseline_acceleration(motor)
        if baseline <= 0:
            return None
        wanted = int(
            round(baseline * min(1.0, change_per_sec / self.acceleration_scale))
        )
        return max(1, min(baseline, wanted))

    def _log_perf_trace(self, line: str) -> None:
        if self.node is not None:
            log_perf_trace(self.node.get_logger(), line)
        else:
            logging.info(line)

    @staticmethod
    def _speed_curve_for(motor: MotorLike) -> tuple[tuple[int, float], ...]:
        """The measured curve of the servo behind this motor, else the family default."""
        getter = getattr(motor, "speed_curve", None)
        if callable(getter):
            curve = getter()
            if curve:
                return tuple(curve)
        return curve_for_family("")

    @staticmethod
    def _baseline_acceleration(motor: MotorLike) -> int:
        getter = getattr(motor, "get_settings", None)
        if not callable(getter):
            return 0
        settings = getter() or {}
        try:
            return int(settings.get("acceleration") or 0)
        except (TypeError, ValueError):
            return 0

    def _log_tick_speeds(
        self,
        names: Sequence[str],
        times: Sequence[float],
        series: Mapping[str, Sequence[float]],
        now: float,
    ) -> None:
        for name in names:
            for motor in self.motor_lookup[name]:
                path_speed = interpolate_joint_velocity_centideg_per_sec(
                    times, series[name], now
                )
                setting = derive_sts_velocity_setting(
                    path_speed,
                    curve=self._speed_curve_for(motor),
                    margin=self.path_speed_margin,
                )
                self._log_perf_trace(
                    format_perf_trace_trajectory_leg_motion(
                        motor.name,
                        active_trajectory_leg_index(times, now),
                        times[0],
                        times[-1],
                        path_speed,
                        setting,
                        0,
                    )
                )

    @staticmethod
    def _stream_set_position(
        motor: MotorLike, position: int, speed: int | None, acc: int | None
    ) -> bool:
        setter = motor.set_position
        try:
            return bool(
                setter(position, bus_priority=BusPriority.STREAM, speed=speed, acc=acc)
            )
        except TypeError:
            try:
                return bool(setter(position, bus_priority=BusPriority.STREAM))
            except TypeError:
                return bool(setter(position))

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

    def _log_error(self, message: str) -> None:
        if self.node is not None:
            self.node.get_logger().error(message)
