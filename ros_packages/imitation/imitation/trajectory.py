"""ROS-independent IK validation and timed trajectory construction."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence


@dataclass(frozen=True)
class TimedWaypoint:
    """One standard full-joint trajectory waypoint."""

    positions: tuple[float, ...]
    time_from_start: float


@dataclass(frozen=True)
class TimedTrajectory:
    """Transport-neutral representation of a JointTrajectory."""

    joint_names: tuple[str, ...]
    waypoints: tuple[TimedWaypoint, ...]


def quaternion_to_rpy_degrees(
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float]:
    """Convert a finite normalized XYZW quaternion to roll/pitch/yaw degrees."""
    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion must contain four values")
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(length) or length <= 1e-12:
        raise ValueError("quaternion must be finite and non-zero")
    x, y, z, w = (value / length for value in (x, y, z, w))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def solve_arm_target(
    kinematics: Any,
    xyz_mm: Sequence[float],
    quaternion_xyzw: Sequence[float],
    *,
    use_orientation: bool = False,
    initial_guess_deg: Sequence[float] | None = None,
) -> tuple[float, ...]:
    """Run the SDK solver and independently enforce its mechanical limits."""
    if len(xyz_mm) != 3 or not all(math.isfinite(float(value)) for value in xyz_mm):
        raise ValueError("IK target must contain three finite coordinates")
    kwargs = {
        "xyz": tuple(float(value) for value in xyz_mm),
        "initial_guess_deg": initial_guess_deg,
        "respect_limits": True,
    }
    if use_orientation:
        kwargs["rpy_deg"] = quaternion_to_rpy_degrees(quaternion_xyzw)
    solution = tuple(float(value) for value in kinematics.inverse(**kwargs))
    motor_names = tuple(kinematics.motor_names)
    if len(solution) != len(motor_names) or not all(map(math.isfinite, solution)):
        raise ValueError("IK returned an invalid joint vector")
    lower, upper = kinematics.joint_limits_deg
    if len(lower) != len(solution) or len(upper) != len(solution):
        raise ValueError("kinematics limits do not match its motor order")
    if any(
        angle < float(low) - 1e-9 or angle > float(high) + 1e-9
        for angle, low, high in zip(solution, lower, upper)
    ):
        raise ValueError("IK solution violates mechanical joint limits")
    return solution


def degrees_to_hundredths(joint_degrees: Sequence[float]) -> tuple[float, ...]:
    """Convert finite joint degrees to the motor controller's internal unit."""
    values = tuple(float(value) for value in joint_degrees)
    if not values or not all(map(math.isfinite, values)):
        raise ValueError("joint target must contain finite values")
    return tuple(float(round(value * 100.0)) for value in values)


def build_timed_trajectory(
    motor_names: Sequence[str],
    previous_degrees: Sequence[float],
    target_degrees: Sequence[float],
    *,
    duration_sec: float,
) -> TimedTrajectory:
    """Build two full-joint waypoints consumed by TrajectoryExecutor."""
    names = tuple(str(name) for name in motor_names)
    previous = degrees_to_hundredths(previous_degrees)
    target = degrees_to_hundredths(target_degrees)
    if not names or len(set(names)) != len(names):
        raise ValueError("motor names must be non-empty and unique")
    if len(names) != len(previous) or len(names) != len(target):
        raise ValueError("each waypoint must contain one position per motor")
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError("trajectory duration must be finite and positive")
    return TimedTrajectory(
        joint_names=names,
        waypoints=(
            TimedWaypoint(previous, 0.0),
            TimedWaypoint(target, float(duration_sec)),
        ),
    )


class TeleopController:
    """Stateful safety gate around IK and trajectory service submission."""

    def __init__(
        self,
        kinematics: Any,
        trajectory_client: Any,
        *,
        enable_motion: bool = False,
        duration_sec: float = 0.25,
        use_orientation: bool = False,
    ) -> None:
        self.kinematics = kinematics
        self.trajectory_client = trajectory_client
        self.enable_motion = bool(enable_motion)
        self.duration_sec = float(duration_sec)
        self.use_orientation = bool(use_orientation)
        self.previous_solution: tuple[float, ...] | None = None
        self.request_in_flight = False

    def solve(
        self, xyz_mm: Sequence[float], quaternion_xyzw: Sequence[float]
    ) -> tuple[float, ...]:
        """Solve a target, using the last valid target as the IK seed."""
        return solve_arm_target(
            self.kinematics,
            xyz_mm,
            quaternion_xyzw,
            use_orientation=self.use_orientation,
            initial_guess_deg=self.previous_solution,
        )

    def prepare(
        self, xyz_mm: Sequence[float], quaternion_xyzw: Sequence[float]
    ) -> TimedTrajectory | None:
        """Return a safe command or only observe/prime when actuation is disabled."""
        solution = self.solve(xyz_mm, quaternion_xyzw)
        previous = self.previous_solution
        self.previous_solution = solution
        if not self.enable_motion or previous is None or self.request_in_flight:
            return None
        return build_timed_trajectory(
            self.kinematics.motor_names,
            previous,
            solution,
            duration_sec=self.duration_sec,
        )

    def submit(self, request: Any) -> Any | None:
        """Submit exactly one request at a time; completion unlocks the gate."""
        if not self.enable_motion or self.request_in_flight:
            return None
        self.request_in_flight = True
        try:
            future = self.trajectory_client.call_async(request)
        except Exception:
            self.request_in_flight = False
            raise

        def complete(_future: Any) -> None:
            self.request_in_flight = False

        future.add_done_callback(complete)
        return future
