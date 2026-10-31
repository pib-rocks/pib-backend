"""Offline unit tests for TrajectoryExecutor (PR-1660).

Interpolation and the executor are ROS-graph-free: tests use duck-typed
trajectory messages and a fake Motor. If the module cannot be imported
(unexpected), skip with a reason instead of failing the rest of the suite.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Any
from queue import SimpleQueue

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_ROOT = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_ROOT) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_ROOT))

try:
    from pib_motors.trajectory_executor import (
        DEFAULT_SHARP_ACCELERATION,
        DEFAULT_SHARP_DECELERATION,
        DEFAULT_SHARP_VELOCITY,
        MAX_SOFTWARE_TRAJECTORY_DURATION_SEC,
        TrajectoryExecutor,
        interpolate_joint,
        interpolate_positions,
        is_software_trajectory,
        validate_software_trajectory_schedule,
    )
except ImportError as exc:  # pragma: no cover - unexpected missing module
    pytest.skip(
        f"trajectory_executor is not importable: {exc}",
        allow_module_level=True,
    )


class FakeDuration:
    def __init__(self, sec: float = 0.0, nanosec: int = 0) -> None:
        whole = int(sec)
        frac = sec - whole
        self.sec = whole
        self.nanosec = nanosec + int(round(frac * 1e9))


class FakePoint:
    def __init__(self, positions: list[float], time_from_start: float = 0.0) -> None:
        self.positions = list(positions)
        self.time_from_start = FakeDuration(time_from_start)


class FakeJointTrajectory:
    def __init__(self, joint_names: list[str], points: list[FakePoint]) -> None:
        self.joint_names = joint_names
        self.points = points


class FakeMotor:
    def __init__(self, name: str, settings: dict[str, Any] | None = None) -> None:
        self.name = name
        self.positions: list[int] = []
        self.settings_calls: list[dict[str, Any]] = []
        self._settings = {
            "name": name,
            "visible": True,
            "upperEndpoint": "max",
            "rotationRangeMin": -9000,
            "rotationRangeMax": 9000,
            "turnedOn": True,
            "velocity": 4000,
            "acceleration": 3000,
            "deceleration": 3000,
            "pulseWidthMin": 1000,
            "pulseWidthMax": 2000,
            "period": 19500,
        }
        if settings:
            self._settings.update(settings)
        self._position = 0

    def set_position(self, position: int) -> bool:
        self.positions.append(int(position))
        self._position = int(position)
        return True

    def get_position(self) -> int:
        return self._position

    def get_settings(self) -> dict[str, Any]:
        return dict(self._settings)

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        self.settings_calls.append(dict(settings_dto))
        self._settings = dict(settings_dto)
        return True

    def check_if_motor_is_connected(self) -> bool:
        return True


class FakeClock:
    """Deterministic clock: ``sleep`` advances time so tests do not wait."""

    def __init__(self) -> None:
        self.now = 0.0

    def time(self) -> float:
        return self.now

    def sleep(self, dt: float) -> None:
        self.now += dt


def _executor(
    motors: dict[str, list[FakeMotor]], clock: FakeClock | None = None
) -> TrajectoryExecutor:
    clock = clock or FakeClock()
    return TrajectoryExecutor(
        node=None,
        motor_lookup=motors,
        rate_hz=10.0,
        sleep_fn=clock.sleep,
        clock_fn=clock.time,
    )


def test_interpolate_joint_is_linear_between_two_points() -> None:
    times = [0.0, 1.0]
    values = [0.0, 100.0]
    assert interpolate_joint(times, values, 0.0) == pytest.approx(0.0)
    assert interpolate_joint(times, values, 0.5) == pytest.approx(50.0)
    assert interpolate_joint(times, values, 1.0) == pytest.approx(100.0)
    assert interpolate_joint(times, values, 1.5) == pytest.approx(100.0)


def test_via_point_mid_segment_is_interpolated_not_clamped() -> None:
    times = [0.0, 1.0, 2.0]
    values = [0.0, 100.0, 200.0]
    mid = interpolate_joint(times, values, 0.5)
    assert 0.0 < mid < 100.0
    after_via = interpolate_joint(times, values, 1.2)
    assert after_via > 100.0
    targets = interpolate_positions(["elbow"], times, {"elbow": values}, 0.5)
    assert 0 < targets["elbow"] < 100


def test_is_software_trajectory_single_point_command_is_not_software() -> None:
    jt = FakeJointTrajectory(["a", "b"], [FakePoint([10.0, 20.0])])
    assert is_software_trajectory(jt) is False


def test_is_software_trajectory_legacy_zip_shape_is_not_software() -> None:
    """Retired N×single-position layout is rejected upstream; not a software trajectory."""
    jt = FakeJointTrajectory(
        ["a", "b"],
        [FakePoint([10]), FakePoint([20])],
    )
    assert is_software_trajectory(jt) is False


def test_is_software_trajectory_multi_point_full_positions() -> None:
    jt = FakeJointTrajectory(
        ["a", "b"],
        [FakePoint([0.0, 10.0], 0.0), FakePoint([5.0, 15.0], 1.0)],
    )
    assert is_software_trajectory(jt) is True


def test_is_software_trajectory_detects_time_from_start() -> None:
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 1.0)],
    )
    assert is_software_trajectory(jt) is True


def test_multi_point_interpolation_calls_set_position_in_order() -> None:
    motor = FakeMotor("elbow")
    clock = FakeClock()
    executor = _executor({"elbow": [motor]}, clock)
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 1.0)],
    )

    assert executor.execute(jt) is True
    assert len(motor.positions) >= 2
    assert motor.positions[0] == 0
    assert motor.positions[-1] == 100
    assert motor.positions == sorted(motor.positions)


def test_motion_config_is_sharpened_then_restored() -> None:
    motor = FakeMotor("elbow")
    original_velocity = motor.get_settings()["velocity"]
    executor = _executor({"elbow": [motor]})
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([50], 0.2)],
    )

    assert executor.execute(jt) is True
    assert len(motor.settings_calls) >= 2
    sharp, restored = motor.settings_calls[0], motor.settings_calls[-1]
    assert sharp["velocity"] == DEFAULT_SHARP_VELOCITY
    assert sharp["acceleration"] == DEFAULT_SHARP_ACCELERATION
    assert sharp["deceleration"] == DEFAULT_SHARP_DECELERATION
    assert restored["velocity"] == original_velocity
    assert restored["acceleration"] == 3000
    assert restored["deceleration"] == 3000
    assert motor.get_settings()["velocity"] == original_velocity


def test_single_point_is_one_set_position_without_sharp_restore() -> None:
    motor = FakeMotor("elbow")
    executor = _executor({"elbow": [motor]})
    jt = FakeJointTrajectory(["elbow"], [FakePoint([42])])

    assert executor.execute(jt) is True
    assert motor.positions == [42]
    assert motor.settings_calls == []


def test_single_point_multi_joint_sets_every_motor() -> None:
    motors = {
        "a": [FakeMotor("a")],
        "b": [FakeMotor("b")],
        "c": [FakeMotor("c")],
    }
    executor = _executor(motors)
    jt = FakeJointTrajectory(
        ["a", "b", "c"],
        [FakePoint([10, 20, 30])],
    )

    assert executor.execute(jt) is True
    assert motors["a"][0].positions == [10]
    assert motors["b"][0].positions == [20]
    assert motors["c"][0].positions == [30]


def test_via_point_blend_does_not_dwell_at_via_point() -> None:
    motor = FakeMotor("elbow")
    executor = _executor({"elbow": [motor]})
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 1.0), FakePoint([200], 2.0)],
    )

    assert executor.execute(jt) is True
    assert any(0 < p < 100 for p in motor.positions), motor.positions
    assert any(100 < p < 200 for p in motor.positions), motor.positions
    # After crossing the via-point the stream must keep moving, not hold 100.
    via_indices = [i for i, p in enumerate(motor.positions) if p == 100]
    assert via_indices
    first_via = via_indices[0]
    tail = motor.positions[first_via + 1 :]
    assert tail, "trajectory ended at the via-point instead of blending through"
    assert any(p != 100 for p in tail)


def test_validate_rejects_zero_second_leg() -> None:
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([50], 1.0), FakePoint([100], 1.0)],
    )
    message = validate_software_trajectory_schedule(jt)
    assert message is not None
    assert "not allowed" in message


def test_validate_rejects_duplicate_waypoint_times() -> None:
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([50], 2.0), FakePoint([100], 2.0)],
    )
    message = validate_software_trajectory_schedule(jt)
    assert message is not None
    assert "duplicate" in message


def test_validate_rejects_duration_above_maximum() -> None:
    over = MAX_SOFTWARE_TRAJECTORY_DURATION_SEC + 1.0
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([10], over)],
    )
    message = validate_software_trajectory_schedule(jt)
    assert message is not None
    assert "maximum" in message


def test_invalid_schedule_execute_does_not_move_motor() -> None:
    motor = FakeMotor("elbow")
    executor = _executor({"elbow": [motor]})
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 0.0)],
    )
    assert executor.execute(jt) is False
    assert motor.positions == []
    assert motor.settings_calls == []


def test_cancel_stops_interpolation_and_restores_settings() -> None:
    motor = FakeMotor("elbow")
    original_velocity = motor.get_settings()["velocity"]
    executor = TrajectoryExecutor(
        node=None,
        motor_lookup={"elbow": [motor]},
        rate_hz=20.0,
    )
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([1000], 2.0)],
    )
    completion_queue: SimpleQueue = SimpleQueue()
    started, _ = executor.start_async(jt, completion_queue)
    assert started is True
    time.sleep(0.05)
    assert executor.cancel() is True
    completion = completion_queue.get(timeout=2.0)
    assert completion.cancelled is True
    assert motor.get_settings()["velocity"] == original_velocity
    assert motor.positions[-1] < 1000


def test_start_async_returns_long_before_the_trajectory_duration_has_elapsed() -> None:
    """Load-bearing: the caller gets control back immediately, not after the motion.

    The sibling test only asserts is_running()/join(). A synchronous implementation
    satisfies both, so this one measures WALL time on a real clock: the loop really
    sleeps for about 0.4 s, and the call must still return in a fraction of that.
    """
    motor = FakeMotor("elbow")
    executor = TrajectoryExecutor(
        None,
        {"elbow": [motor]},
        rate_hz=20.0,
        sleep_fn=time.sleep,
        clock_fn=time.monotonic,
    )
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 0.4)],
    )

    started_at = time.monotonic()
    started, _error = executor.start_async(jt, SimpleQueue())
    elapsed = time.monotonic() - started_at

    assert started is True
    assert (
        elapsed < 0.1
    ), f"start_async blocked for {elapsed:.2f} s (trajectory is 0.4 s)"
    assert executor.is_running() is True
    assert executor.join(timeout=10.0) is True
    assert motor.positions, "the worker never commanded the motor"


def test_start_async_returns_before_duration_elapses() -> None:
    motor = FakeMotor("elbow")
    clock = FakeClock()
    executor = _executor({"elbow": [motor]}, clock)
    jt = FakeJointTrajectory(
        ["elbow"],
        [FakePoint([0], 0.0), FakePoint([100], 5.0)],
    )
    started, _ = executor.start_async(jt, SimpleQueue())
    assert started is True
    assert executor.is_running() is True
    assert executor.join(timeout=1.0) is True
