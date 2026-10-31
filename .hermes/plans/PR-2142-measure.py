#!/usr/bin/env python3
"""Live robot measurement for PR-2142 path-coupled servo motion (ros-motors container).

Usage:
  source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash
  python3 .hermes/plans/PR-2142-measure.py [--joint NAME] [--amplitude-rad A] [--duration-sec T]
  python3 .hermes/plans/PR-2142-measure.py --multi-adapters
  python3 .hermes/plans/PR-2142-measure.py --calibrate-speed --joint NAME --sts-speed 1000

Prints one JSON object per sequence plus a summary line. Exits non-zero when a criterion
is violated. Does not embed lab measurement results.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import rclpy
from datatypes.srv import ApplyJointTrajectory, GetJointPosition
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Reviewer gates (not pre-filled pass/fail from this workspace).
MAX_JOINT_STATE_GAP_MS = 250.0
MAX_LARGEST_TO_MEDIAN_STEP_RATIO = 8.0
MAX_QUIET_PHASE_MS = 200.0
MAX_FINAL_POSITION_ERROR_RAD = 0.02
DEFAULT_DURATION_SEC = 10.0
DEFAULT_AMPLITUDE_RAD = 0.3
MULTI_AMPLITUDE_RAD = 0.02
MULTI_DURATION_SEC = 10.0

# Two joints per Waveshare adapter on pib5advanced (controller 1..4).
MULTI_ADAPTER_JOINTS: tuple[str, ...] = (
    "turn_head_motor",
    "shoulder_vertical_right",
    "elbow_right",
    "upper_arm_right_rotation",
    "tilt_forward_motor",
    "shoulder_vertical_left",
    "elbow_left",
    "upper_arm_left_rotation",
)


def _json(phase: str, **fields: Any) -> None:
    payload = {"phase": phase, **fields}
    print(json.dumps(payload), flush=True)


def _rad_from_joint_state(msg: JointState, joint_name: str) -> float | None:
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        return None
    if index >= len(msg.position):
        return None
    return float(msg.position[index])


@dataclass
class JointSampleTrace:
    joint_name: str
    samples: list[tuple[float, float]] = field(default_factory=list)
    receive_times: list[float] = field(default_factory=list)

    def record(self, wall_time: float, position_rad: float) -> None:
        self.samples.append((wall_time, position_rad))
        self.receive_times.append(wall_time)

    def step_stats(self) -> dict[str, float]:
        if len(self.samples) < 2:
            return {
                "median_step_rad": 0.0,
                "largest_step_rad": 0.0,
                "largest_to_median_ratio": 0.0,
            }
        steps = [
            abs(self.samples[i][1] - self.samples[i - 1][1])
            for i in range(1, len(self.samples))
        ]
        steps_sorted = sorted(steps)
        median = steps_sorted[len(steps_sorted) // 2]
        largest = max(steps)
        ratio = largest / median if median > 0.0 else float("inf")
        return {
            "median_step_rad": median,
            "largest_step_rad": largest,
            "largest_to_median_ratio": ratio,
        }

    def longest_quiet_phase_ms(
        self, t_start: float, t_end: float, epsilon_rad: float = 1e-5
    ) -> float:
        window = [
            (t, p) for t, p in self.samples if t_start <= t <= t_end and t_end > t_start
        ]
        if len(window) < 2:
            return 0.0
        longest = 0.0
        quiet_start: float | None = None
        ref = window[0][1]
        for t, pos in window[1:]:
            if abs(pos - ref) <= epsilon_rad:
                if quiet_start is None:
                    quiet_start = t
            else:
                if quiet_start is not None:
                    longest = max(longest, (t - quiet_start) * 1000.0)
                quiet_start = None
                ref = pos
        if quiet_start is not None:
            longest = max(longest, (window[-1][0] - quiet_start) * 1000.0)
        return longest

    def joint_state_rate_hz(self) -> float:
        if len(self.receive_times) < 2:
            return 0.0
        span = self.receive_times[-1] - self.receive_times[0]
        if span <= 0.0:
            return 0.0
        return (len(self.receive_times) - 1) / span

    def max_joint_state_gap_ms(self) -> float:
        if len(self.receive_times) < 2:
            return 0.0
        return max(
            (self.receive_times[i] - self.receive_times[i - 1]) * 1000.0
            for i in range(1, len(self.receive_times))
        )


class MeasureNode(Node):
    def __init__(self, joint_names: Sequence[str]) -> None:
        super().__init__("pr_2142_measure")
        self.joint_names = list(joint_names)
        self.traces = {name: JointSampleTrace(name) for name in self.joint_names}
        self._motion_started: float | None = None
        self._motion_duration = 0.0
        self._target_end_rad: dict[str, float] = {}
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self._apply = self.create_client(ApplyJointTrajectory, "apply_joint_trajectory")
        self._get_pos = self.create_client(GetJointPosition, "get_joint_position")

    def _on_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        for name in self.joint_names:
            rad = _rad_from_joint_state(msg, name)
            if rad is None:
                continue
            if self._motion_started is not None:
                self.traces[name].record(now, rad)

    def _wait_services(self, timeout_sec: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        for client in (self._apply, self._get_pos):
            while not client.wait_for_service(timeout_sec=0.2):
                if time.monotonic() > deadline:
                    return False
        return True

    def _read_joint_rad(self, joint_name: str) -> tuple[float, float]:
        request = GetJointPosition.Request()
        request.joint_name = joint_name
        started = time.monotonic()
        future = self._get_pos.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        latency_ms = (time.monotonic() - started) * 1000.0
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError(
                f"get_joint_position failed for {joint_name}: "
                f"{getattr(response, 'message', 'no response')}"
            )
        return float(response.position), latency_ms

    def _build_trajectory(
        self,
        start_rad_by_joint: dict[str, float],
        amplitude_rad: float,
        duration_sec: float,
    ) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)
        p0 = JointTrajectoryPoint()
        p0.positions = [start_rad_by_joint[name] for name in jt.joint_names]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [
            start_rad_by_joint[name] + amplitude_rad for name in jt.joint_names
        ]
        half = duration_sec / 2.0
        p1.time_from_start.sec = int(half)
        p1.time_from_start.nanosec = int(round((half - int(half)) * 1e9))
        p2 = JointTrajectoryPoint()
        p2.positions = [start_rad_by_joint[name] for name in jt.joint_names]
        p2.time_from_start.sec = int(duration_sec)
        p2.time_from_start.nanosec = int(
            round((duration_sec - int(duration_sec)) * 1e9)
        )
        jt.points = [p0, p1, p2]
        for name in jt.joint_names:
            self._target_end_rad[name] = start_rad_by_joint[name]
        return jt

    def _run_trajectory(self, jt: JointTrajectory, duration_sec: float) -> float:
        request = ApplyJointTrajectory.Request()
        request.joint_trajectory = jt
        started = time.monotonic()
        future = self._apply.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        apply_ms = (time.monotonic() - started) * 1000.0
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError("apply_joint_trajectory rejected the motion")
        self._motion_started = time.monotonic()
        self._motion_duration = duration_sec
        deadline = self._motion_started + duration_sec + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return apply_ms

    def _restore_hold(self, start_rad_by_joint: dict[str, float]) -> None:
        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [start_rad_by_joint[name] for name in jt.joint_names]
        point.time_from_start.sec = 0
        jt.points = [point]
        request = ApplyJointTrajectory.Request()
        request.joint_trajectory = jt
        future = self._apply.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

    def _evaluate(self, violations: list[str]) -> None:
        t0 = self._motion_started or 0.0
        t_start = t0 + 0.05 * self._motion_duration
        t_end = t0 + 0.95 * self._motion_duration
        for name, trace in self.traces.items():
            steps = trace.step_stats()
            quiet_ms = trace.longest_quiet_phase_ms(t_start, t_end)
            rate_hz = trace.joint_state_rate_hz()
            gap_ms = trace.max_joint_state_gap_ms()
            final_rad = trace.samples[-1][1] if trace.samples else float("nan")
            target_rad = self._target_end_rad.get(name, float("nan"))
            final_error_rad = abs(final_rad - target_rad)
            payload = {
                "joint": name,
                **{
                    k: round(v, 6) if isinstance(v, float) else v
                    for k, v in steps.items()
                },
                "longest_quiet_phase_ms": round(quiet_ms, 2),
                "final_error_rad": round(final_error_rad, 6),
                "joint_states_hz": round(rate_hz, 2),
                "joint_states_max_gap_ms": round(gap_ms, 2),
            }
            _json("sequence", **payload)
            if steps["largest_to_median_ratio"] > MAX_LARGEST_TO_MEDIAN_STEP_RATIO:
                violations.append(
                    f"{name}: step ratio {steps['largest_to_median_ratio']:.2f} "
                    f"> {MAX_LARGEST_TO_MEDIAN_STEP_RATIO}"
                )
            if quiet_ms > MAX_QUIET_PHASE_MS:
                violations.append(
                    f"{name}: quiet phase {quiet_ms:.1f}ms > {MAX_QUIET_PHASE_MS}ms"
                )
            if final_error_rad > MAX_FINAL_POSITION_ERROR_RAD:
                violations.append(
                    f"{name}: final error {final_error_rad:.4f} rad "
                    f"> {MAX_FINAL_POSITION_ERROR_RAD} rad"
                )
            if gap_ms > MAX_JOINT_STATE_GAP_MS:
                violations.append(
                    f"{name}: joint_states gap {gap_ms:.1f}ms > {MAX_JOINT_STATE_GAP_MS}ms"
                )

    def run_motion(
        self,
        amplitude_rad: float,
        duration_sec: float,
    ) -> int:
        if not self._wait_services():
            _json("error", message="ROS services not available")
            return 2

        start_rad_by_joint: dict[str, float] = {}
        for name in self.joint_names:
            start_rad, latency_ms = self._read_joint_rad(name)
            start_rad_by_joint[name] = start_rad
            _json(
                "baseline",
                joint=name,
                start_rad=round(start_rad, 6),
                get_position_ms=round(latency_ms, 2),
            )

        violations: list[str] = []
        jt = self._build_trajectory(start_rad_by_joint, amplitude_rad, duration_sec)
        apply_ms = self._run_trajectory(jt, duration_sec)
        _json("trajectory_done", apply_ms=round(apply_ms, 2))
        self._evaluate(violations)
        self._restore_hold(start_rad_by_joint)
        _json("restore", joints=self.joint_names)

        summary = {
            "phase": "summary",
            "joints": self.joint_names,
            "amplitude_rad": amplitude_rad,
            "duration_sec": duration_sec,
            "violations": violations,
            "pass": not violations,
        }
        print(json.dumps(summary), flush=True)
        return 0 if not violations else 1

    def run_calibrate_speed(
        self, joint_name: str, sts_speed: int, move_rad: float
    ) -> int:
        if not self._wait_services():
            _json("error", message="ROS services not available")
            return 2
        if sts_speed < 1 or sts_speed > 3400:
            _json("error", message="sts_speed must be between 1 and 3400")
            return 2

        self.joint_names = [joint_name]
        self.traces = {joint_name: JointSampleTrace(joint_name)}
        start_rad, _ = self._read_joint_rad(joint_name)
        target_rad = start_rad + move_rad
        jt = JointTrajectory()
        jt.joint_names = [joint_name]
        p0 = JointTrajectoryPoint()
        p0.positions = [start_rad]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [target_rad]
        p1.time_from_start.sec = 8
        jt.points = [p0, p1]

        # Direct velocity on the motor is outside this script; reviewer sets sts_speed in
        # motor settings UI or a one-off service call before running. This probe assumes
        # velocity already equals sts_speed in motor settings.
        _json(
            "calibrate_start",
            joint=joint_name,
            sts_speed=sts_speed,
            start_rad=round(start_rad, 6),
            target_rad=round(target_rad, 6),
            note="Set motor velocity to sts_speed before measuring.",
        )
        self._target_end_rad[joint_name] = target_rad
        self._run_trajectory(jt, 8.0)

        trace = self.traces[joint_name]
        if len(trace.samples) < 5:
            _json(
                "error",
                message="not enough joint_states samples during calibration move",
            )
            return 2

        t0, p0s = trace.samples[0]
        t1, p1s = trace.samples[-1]
        dt = t1 - t0
        if dt <= 0.0:
            _json("error", message="non-positive calibration duration")
            return 2
        delta_centideg = abs(p1s - p0s) * 18000.0 / math.pi
        centideg_per_sec = delta_centideg / dt
        factor = centideg_per_sec / float(sts_speed)
        _json(
            "calibrate_result",
            joint=joint_name,
            sts_speed=sts_speed,
            duration_sec=round(dt, 3),
            delta_rad=round(abs(p1s - p0s), 6),
            centideg_per_sec=round(centideg_per_sec, 4),
            STS_CENTIDEG_PER_SEC_PER_SPEED_UNIT=round(factor, 6),
        )
        self._restore_hold({joint_name: start_rad})
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="PR-2142 smooth trajectory measurement"
    )
    parser.add_argument(
        "--joint", default="elbow_right", help="Joint for single-joint run"
    )
    parser.add_argument(
        "--amplitude-rad",
        type=float,
        default=DEFAULT_AMPLITUDE_RAD,
        help="Peak deviation from start (single-joint default 0.3 rad)",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=DEFAULT_DURATION_SEC,
        help="Trajectory duration (default 10 s)",
    )
    parser.add_argument(
        "--multi-adapters",
        action="store_true",
        help="Run eight joints (two per serial adapter) with 0.02 rad over 10 s",
    )
    parser.add_argument(
        "--calibrate-speed",
        action="store_true",
        help="Estimate STS_CENTIDEG_PER_SEC_PER_SPEED_UNIT for one joint",
    )
    parser.add_argument(
        "--sts-speed",
        type=int,
        default=1000,
        help="Motor velocity setting used during --calibrate-speed",
    )
    parser.add_argument(
        "--calibrate-move-rad",
        type=float,
        default=0.25,
        help="Calibration move amplitude in rad",
    )
    args = parser.parse_args()

    rclpy.init()
    if args.calibrate_speed:
        node = MeasureNode([args.joint])
        try:
            return node.run_calibrate_speed(
                args.joint, args.sts_speed, args.calibrate_move_rad
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()

    joints = list(MULTI_ADAPTER_JOINTS) if args.multi_adapters else [args.joint]
    amplitude = MULTI_AMPLITUDE_RAD if args.multi_adapters else args.amplitude_rad
    duration = MULTI_DURATION_SEC if args.multi_adapters else args.duration_sec
    node = MeasureNode(joints)
    try:
        return node.run_motion(amplitude, duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
