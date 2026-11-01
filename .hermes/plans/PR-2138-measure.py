#!/usr/bin/env python3
"""Live robot measurement for PR-2138 bus writer (run inside ros-motors container).

Usage (from task prompt):
  source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash
  python3 .hermes/plans/PR-2138-measure.py [--joint NAME] [--nopoll]

Prints one JSON object per phase plus a summary line. Exits non-zero when a
criterion is violated. Does not embed measured numbers from lab runs.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

import rclpy
from datatypes.srv import ApplyJointTrajectory, GetJointPosition
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Pass/fail gates for the reviewer on hardware (not pre-filled results).
MAX_JOINT_STATE_GAP_MS = 750.0
MAX_GET_POSITION_P99_MS = 500.0
MAX_CANCEL_ACK_MS = 100.0
TRAJECTORY_DURATION_SEC = 6.0
AMPLITUDE_RAD = 0.02
POLL_HZ = 20.0


def _json(phase: str, **fields: Any) -> None:
    payload = {"phase": phase, **fields}
    print(json.dumps(payload), flush=True)


class MeasureNode(Node):
    def __init__(self, joint_name: str, nopoll: bool) -> None:
        super().__init__("pr_2138_measure")
        self.joint_name = joint_name
        self.nopoll = nopoll
        self._last_joint_state_mono: float | None = None
        self._max_joint_state_gap_ms = 0.0
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self._apply = self.create_client(ApplyJointTrajectory, "apply_joint_trajectory")
        self._get_pos = self.create_client(GetJointPosition, "get_joint_position")

    def _on_joint_state(self, msg: JointState) -> None:
        if self.joint_name not in msg.name:
            return
        now = time.monotonic()
        if self._last_joint_state_mono is not None:
            gap_ms = (now - self._last_joint_state_mono) * 1000.0
            if gap_ms > self._max_joint_state_gap_ms:
                self._max_joint_state_gap_ms = gap_ms
        self._last_joint_state_mono = now

    def _wait_services(self, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        for client in (self._apply, self._get_pos):
            while not client.wait_for_service(timeout_sec=0.2):
                if time.monotonic() > deadline:
                    return False
        return True

    def _read_joint_rad(self) -> float:
        request = GetJointPosition.Request()
        request.joint_name = self.joint_name
        started = time.monotonic()
        future = self._get_pos.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        latency_ms = (time.monotonic() - started) * 1000.0
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError(
                f"get_joint_position failed: {getattr(response, 'message', 'no response')}"
            )
        return float(response.position), latency_ms

    def _build_trajectory(self, start_rad: float) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = [self.joint_name]
        p0 = JointTrajectoryPoint()
        p0.positions = [start_rad]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [start_rad + AMPLITUDE_RAD]
        p1.time_from_start.sec = int(TRAJECTORY_DURATION_SEC / 2)
        p2 = JointTrajectoryPoint()
        p2.positions = [start_rad]
        p2.time_from_start.sec = int(TRAJECTORY_DURATION_SEC)
        jt.points = [p0, p1, p2]
        return jt

    def run(self) -> int:
        if not self._wait_services():
            _json("error", message="ROS services not available")
            return 2

        start_rad, baseline_latency_ms = self._read_joint_rad()
        _json(
            "baseline",
            joint=self.joint_name,
            start_rad=start_rad,
            get_position_ms=round(baseline_latency_ms, 2),
        )

        violations: list[str] = []
        poll_latencies_ms: list[float] = []

        jt = self._build_trajectory(start_rad)
        apply_req = ApplyJointTrajectory.Request()
        apply_req.joint_trajectory = jt
        apply_started = time.monotonic()
        apply_future = self._apply.call_async(apply_req)
        while not apply_future.done():
            if not self.nopoll:
                try:
                    _, latency_ms = self._read_joint_rad()
                    poll_latencies_ms.append(latency_ms)
                except RuntimeError as error:
                    _json("poll_error", message=str(error))
            time.sleep(max(0.0, 1.0 / POLL_HZ - 0.001))
            rclpy.spin_once(self, timeout_sec=0.0)
        apply_latency_ms = (time.monotonic() - apply_started) * 1000.0
        apply_response = apply_future.result()
        _json(
            "trajectory_start",
            apply_ms=round(apply_latency_ms, 2),
            successful=bool(getattr(apply_response, "successful", False)),
        )

        cancel_deadline = time.monotonic() + 1.0
        while time.monotonic() < cancel_deadline:
            if not self.nopoll:
                try:
                    _, latency_ms = self._read_joint_rad()
                    poll_latencies_ms.append(latency_ms)
                except RuntimeError as error:
                    _json("poll_error", message=str(error))
            time.sleep(max(0.0, 1.0 / POLL_HZ - 0.001))
            rclpy.spin_once(self, timeout_sec=0.0)

        cancel_jt = JointTrajectory()
        cancel_req = ApplyJointTrajectory.Request()
        cancel_req.joint_trajectory = cancel_jt
        cancel_started = time.monotonic()
        cancel_future = self._apply.call_async(cancel_req)
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
        cancel_ack_ms = (time.monotonic() - cancel_started) * 1000.0
        cancel_response = cancel_future.result()
        _json(
            "cancel",
            ack_ms=round(cancel_ack_ms, 2),
            successful=bool(getattr(cancel_response, "successful", False)),
        )
        if cancel_ack_ms > MAX_CANCEL_ACK_MS:
            violations.append(
                f"cancel ack {cancel_ack_ms:.1f}ms > {MAX_CANCEL_ACK_MS}ms"
            )

        motion_end = time.monotonic() + 1.0
        while time.monotonic() < motion_end:
            if not self.nopoll:
                try:
                    _, latency_ms = self._read_joint_rad()
                    poll_latencies_ms.append(latency_ms)
                except RuntimeError as error:
                    _json("poll_error", message=str(error))
            time.sleep(max(0.0, 1.0 / POLL_HZ - 0.001))
            rclpy.spin_once(self, timeout_sec=0.0)

        if poll_latencies_ms:
            poll_latencies_ms.sort()
            p99_index = max(0, int(math.ceil(0.99 * len(poll_latencies_ms))) - 1)
            p99_ms = poll_latencies_ms[p99_index]
            _json(
                "poll_stats",
                samples=len(poll_latencies_ms),
                p99_ms=round(p99_ms, 2),
                max_ms=round(max(poll_latencies_ms), 2),
            )
            if p99_ms > MAX_GET_POSITION_P99_MS:
                violations.append(
                    f"get_joint_position p99 {p99_ms:.1f}ms > {MAX_GET_POSITION_P99_MS}ms"
                )

        _json(
            "joint_states",
            max_gap_ms=round(self._max_joint_state_gap_ms, 2),
        )
        if self._max_joint_state_gap_ms > MAX_JOINT_STATE_GAP_MS:
            violations.append(
                f"joint_states gap {self._max_joint_state_gap_ms:.1f}ms "
                f"> {MAX_JOINT_STATE_GAP_MS}ms"
            )

        restore_jt = self._build_trajectory(start_rad)
        restore_jt.points = restore_jt.points[:1]
        restore_req = ApplyJointTrajectory.Request()
        restore_req.joint_trajectory = restore_jt
        restore_future = self._apply.call_async(restore_req)
        rclpy.spin_until_future_complete(self, restore_future, timeout_sec=10.0)
        _json("restore", successful=bool(restore_future.result().successful))

        summary = {
            "phase": "summary",
            "joint": self.joint_name,
            "nopoll": self.nopoll,
            "violations": violations,
            "pass": not violations,
        }
        print(json.dumps(summary), flush=True)
        return 0 if not violations else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="PR-2138 bus writer measurement")
    parser.add_argument(
        "--joint",
        default="right_elbow_flex",
        help="Joint name for the 6s probe trajectory",
    )
    parser.add_argument(
        "--nopoll",
        action="store_true",
        help="Do not call get_joint_position during motion",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MeasureNode(args.joint, args.nopoll)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
