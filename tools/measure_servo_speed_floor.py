#!/usr/bin/env python3
"""Measure STS low-speed floor and micro-moves for PR-2153 (ros-motors container).

Run on hardware after motor power is on and ``/joint_states`` is publishing non-zero positions
(avoid the first ~8 s after an SSR cycle — zeros there are a known publishing bug).

Usage::

    source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash
    python3 tools/measure_servo_speed_floor.py --joint turn_head_motor --family sts3095
    python3 tools/measure_servo_speed_floor.py --joint index_right_stretch --family finger \\
        --runs 3 --poll-hz 30

Uses ``apply_motor_settings`` (not HTTP PUT) and single-point ``apply_joint_trajectory`` moves.
Position is sampled via ``get_joint_position`` at ``--poll-hz`` (default 30 Hz); ``/joint_states``
is optional diagnostic only (~10 Hz, ~200 ms slope smoothing).

Exits non-zero when any recording is empty, all zeros, or setting 0 is probed without
``--probe-setting-zero``. Prints paste-ready rows for ``servo_speed_curve.py``.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from datatypes.msg import MotorSettings
from datatypes.srv import ApplyJointTrajectory, ApplyMotorSettings, GetJointPosition
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_REPO = __import__("pathlib").Path(__file__).resolve().parents[1]
_PIB_MOTORS = _REPO / "ros_packages" / "motors" / "pib_motors"
if str(_PIB_MOTORS) not in sys.path:
    sys.path.insert(0, str(_PIB_MOTORS))

from pib_motors.servo_speed_curve import (  # noqa: E402
    LOW_SPEED_CALIBRATION_SETTINGS,
    SERVO_FAMILY_FINGER,
    SERVO_FAMILY_STS3095,
    STS_SPEED_SETTING_ZERO_FORBIDDEN,
    rad_per_sec_from_centideg_per_sec,
)
from pib_motors.servo_speed_measurement import (  # noqa: E402
    MICRO_MOVE_REPEATS,
    MICRO_MOVE_TICKS,
    assert_recording_valid,
    centideg_delta_for_ticks,
    centideg_to_rad,
    cruise_slope_centideg_per_sec,
    position_step_ratio,
    suggest_continuous_floor_cdps,
    summarize_cruise_rates,
)

DEFAULT_MOVE_RAD = 0.5
DEFAULT_RUNS = 3
DEFAULT_POLL_HZ = 30.0


def _json(phase: str, **fields: Any) -> None:
    print(json.dumps({"phase": phase, **fields}), flush=True)


@dataclass
class PollTrace:
    samples: list[tuple[float, float]] = field(default_factory=list)

    def record(self, wall_time: float, position_rad: float) -> None:
        self.samples.append((wall_time, position_rad))


class MeasureServoSpeedFloorNode(Node):
    def __init__(self, joint_name: str) -> None:
        super().__init__("measure_servo_speed_floor")
        self.joint_name = joint_name
        self._baseline_settings: MotorSettings | None = None
        self.create_subscription(
            MotorSettings, "motor_settings", self._on_motor_settings, 10
        )
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self._joint_state_hz_samples: list[float] = []
        self._last_joint_state: float | None = None
        self._apply_settings = self.create_client(
            ApplyMotorSettings, "apply_motor_settings"
        )
        self._apply_traj = self.create_client(
            ApplyJointTrajectory, "apply_joint_trajectory"
        )
        self._get_pos = self.create_client(GetJointPosition, "get_joint_position")

    def _on_motor_settings(self, msg: MotorSettings) -> None:
        if msg.motor_name == self.joint_name:
            self._baseline_settings = msg

    def _on_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        if self._last_joint_state is not None:
            gap = now - self._last_joint_state
            if gap > 0.0:
                self._joint_state_hz_samples.append(1.0 / gap)
        self._last_joint_state = now

    def _wait_services(self, timeout_sec: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        for client in (self._apply_settings, self._apply_traj, self._get_pos):
            while not client.wait_for_service(timeout_sec=0.2):
                if time.monotonic() > deadline:
                    return False
        return True

    def _wait_baseline_settings(self, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self._baseline_settings is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._baseline_settings is not None

    def _read_joint_rad(self) -> float:
        request = GetJointPosition.Request()
        request.joint_name = self.joint_name
        future = self._get_pos.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError(
                f"get_joint_position failed: {getattr(response, 'message', 'no response')}"
            )
        return float(response.position)

    def _apply_velocity_setting(self, velocity: int) -> None:
        if self._baseline_settings is None:
            raise RuntimeError(
                "baseline motor settings not received from motor_settings topic"
            )
        if velocity == 0 and STS_SPEED_SETTING_ZERO_FORBIDDEN:
            raise RuntimeError(
                "refusing to apply velocity 0 until STS_SPEED_SETTING_ZERO_MEASURED_BEHAVIOR "
                "is characterized; pass --probe-setting-zero to override"
            )
        settings = MotorSettings()
        base = self._baseline_settings
        settings.motor_name = base.motor_name
        settings.turned_on = base.turned_on
        settings.pulse_width_min = base.pulse_width_min
        settings.pulse_width_max = base.pulse_width_max
        settings.rotation_range_min = base.rotation_range_min
        settings.rotation_range_max = base.rotation_range_max
        settings.velocity = int(velocity)
        settings.acceleration = base.acceleration
        settings.deceleration = base.deceleration
        settings.period = base.period
        settings.visible = base.visible
        settings.upper_endpoint = base.upper_endpoint
        request = ApplyMotorSettings.Request()
        request.motor_settings = settings
        future = self._apply_settings.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        response = future.result()
        if response is None or not response.settings_applied:
            raise RuntimeError(f"apply_motor_settings failed for velocity={velocity}")

    def _single_point_move_rad(self, target_rad: float) -> None:
        jt = JointTrajectory()
        jt.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start.sec = 0
        jt.points = [point]
        request = ApplyJointTrajectory.Request()
        request.joint_trajectory = jt
        future = self._apply_traj.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError("apply_joint_trajectory rejected the move")

    def _poll_move(
        self,
        target_rad: float,
        poll_hz: float,
        settle_sec: float,
    ) -> PollTrace:
        trace = PollTrace()
        interval = 1.0 / max(1.0, poll_hz)
        started = time.monotonic()
        self._single_point_move_rad(target_rad)
        deadline = started + settle_sec
        while time.monotonic() < deadline:
            pos = self._read_joint_rad()
            trace.record(time.monotonic(), pos)
            time.sleep(interval)
        return trace

    def measure_setting(
        self,
        setting: int,
        move_rad: float,
        poll_hz: float,
        runs: int,
    ) -> tuple[list[float], list[float], list[str]]:
        errors: list[str] = []
        cruise_rates: list[float] = []
        step_ratios: list[float] = []
        for run_index in range(runs):
            start_rad = self._read_joint_rad()
            target_rad = start_rad + move_rad
            self._apply_velocity_setting(setting)
            settle = max(8.0, abs(move_rad) / 0.05)
            trace = self._poll_move(target_rad, poll_hz, settle)
            label = f"setting={setting} run={run_index + 1}"
            invalid = assert_recording_valid(trace.samples, label=label)
            if invalid:
                errors.append(invalid)
                continue
            cruise = cruise_slope_centideg_per_sec(trace.samples)
            steps = position_step_ratio(trace.samples)
            _json(
                "speed_run",
                joint=self.joint_name,
                setting=setting,
                run=run_index + 1,
                poll_hz=poll_hz,
                start_rad=round(start_rad, 6),
                target_rad=round(target_rad, 6),
                cruise_cdps=round(cruise, 4),
                **{k: round(v, 6) for k, v in steps.items()},
            )
            cruise_rates.append(cruise)
            step_ratios.append(steps["largest_to_median_ratio"])
            self._single_point_move_rad(start_rad)
            time.sleep(0.3)
        return cruise_rates, step_ratios, errors

    def measure_micro_moves(
        self,
        setting: int,
        poll_hz: float,
    ) -> dict[int, dict[str, Any]]:
        results: dict[int, dict[str, Any]] = {}
        self._apply_velocity_setting(setting)
        for ticks in MICRO_MOVE_TICKS:
            delta_rad = centideg_to_rad(centideg_delta_for_ticks(ticks))
            arrivals = 0
            for attempt in range(MICRO_MOVE_REPEATS):
                start_rad = self._read_joint_rad()
                target_rad = start_rad + delta_rad
                trace = self._poll_move(target_rad, poll_hz, 2.0)
                if assert_recording_valid(
                    trace.samples, f"micro_{ticks}_run_{attempt}"
                ):
                    continue
                final_rad = trace.samples[-1][1]
                if abs(final_rad - target_rad) <= abs(delta_rad) * 0.6:
                    arrivals += 1
                self._single_point_move_rad(start_rad)
                time.sleep(0.15)
            results[ticks] = {
                "attempts": MICRO_MOVE_REPEATS,
                "arrivals": arrivals,
                "delta_rad": delta_rad,
            }
            _json(
                "micro_move",
                joint=self.joint_name,
                setting=setting,
                ticks=ticks,
                arrivals=arrivals,
                attempts=MICRO_MOVE_REPEATS,
                delta_rad=round(delta_rad, 6),
            )
        return results


def _paste_rows(family: str, table: dict[int, dict[str, float]]) -> None:
    _json("paste_header", family=family, file="servo_speed_curve.py")
    for setting in LOW_SPEED_CALIBRATION_SETTINGS:
        stats = table.get(setting, {})
        mean = stats.get("mean_cdps")
        if mean is None:
            line = f"        ({setting}, None),"
        else:
            line = (
                f"        ({setting}, {mean:.4f}),  # runs={int(stats.get('runs', 0))}"
            )
        print(line, flush=True)


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", default="turn_head_motor")
    parser.add_argument(
        "--family",
        choices=(SERVO_FAMILY_STS3095, SERVO_FAMILY_FINGER),
        default=SERVO_FAMILY_STS3095,
    )
    parser.add_argument("--move-rad", type=float, default=DEFAULT_MOVE_RAD)
    parser.add_argument("--runs", type=int, default=DEFAULT_RUNS)
    parser.add_argument("--poll-hz", type=float, default=DEFAULT_POLL_HZ)
    parser.add_argument(
        "--settings",
        type=int,
        nargs="*",
        default=list(LOW_SPEED_CALIBRATION_SETTINGS),
    )
    parser.add_argument(
        "--probe-setting-zero",
        action="store_true",
        help="Include a guarded test of velocity setting 0 (can mean max speed on some firmware).",
    )
    parser.add_argument(
        "--skip-micro-moves",
        action="store_true",
        help="Skip tick micro-move reliability section",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = MeasureServoSpeedFloorNode(args.joint)
    violations: list[str] = []
    try:
        if not node._wait_services():
            _json("error", message="ROS services not available")
            return 2
        if not node._wait_baseline_settings():
            _json(
                "error",
                message=(
                    "no motor_settings message for this joint — start motor_control or "
                    "touch settings once via apply_motor_settings"
                ),
            )
            return 2

        if args.probe_setting_zero:
            import pib_motors.servo_speed_curve as curve_mod

            curve_mod.STS_SPEED_SETTING_ZERO_FORBIDDEN = False

        table: dict[int, dict[str, float]] = {}
        for setting in args.settings:
            if setting == 0 and not args.probe_setting_zero:
                _json("skip", setting=0, reason="use --probe-setting-zero")
                continue
            rates, step_ratios, errors = node.measure_setting(
                setting, args.move_rad, args.poll_hz, args.runs
            )
            violations.extend(errors)
            stats = summarize_cruise_rates(rates)
            stats["runs"] = float(len(rates))
            if step_ratios:
                stats["worst_step_ratio"] = max(step_ratios)
            table[setting] = stats
            _json(
                "setting_summary",
                joint=args.joint,
                family=args.family,
                setting=setting,
                **{k: round(v, 4) for k, v in stats.items()},
            )

        floor_setting, floor_cdps = suggest_continuous_floor_cdps(table)
        if floor_setting is not None and floor_cdps is not None:
            _json(
                "suggested_continuous_floor",
                family=args.family,
                setting=floor_setting,
                floor_cdps=round(floor_cdps, 4),
                floor_rad_s=round(rad_per_sec_from_centideg_per_sec(floor_cdps), 6),
                paste_min_continuous=(
                    f'MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY["{args.family}"] = '
                    f"{floor_cdps:.4f}"
                ),
            )

        if not args.skip_micro_moves and floor_setting is not None:
            node.measure_micro_moves(int(floor_setting), args.poll_hz)

        if node._baseline_settings is not None:
            node._apply_velocity_setting(int(node._baseline_settings.velocity))

        if node._joint_state_hz_samples:
            hz = statistics.mean(node._joint_state_hz_samples)
            _json(
                "joint_states_diag",
                approx_hz=round(hz, 2),
                note="/joint_states is not used for cruise slope; poll-hz drives the table",
            )

        _paste_rows(args.family, table)
        _json(
            "summary",
            joint=args.joint,
            family=args.family,
            violations=violations,
            ok=not violations,
            pending=(
                "Set MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY and LOW_SPEED_CURVE "
                "None rows from the table above"
            ),
        )
        return 0 if not violations else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(run())
