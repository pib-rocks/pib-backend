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
import urllib.request
import sys
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from datatypes.msg import MotorSettings
from datatypes.srv import ApplyJointTrajectory, ApplyMotorSettings, GetJointPosition
from pib_motors.units import centideg_to_rad
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
    def __init__(self, joint_name: str, move_duration_sec: float = 3.0) -> None:
        super().__init__("measure_servo_speed_floor")
        self.joint_name = joint_name
        self._move_duration_sec = move_duration_sec
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

    def _baseline_from_api(self, api_base: str) -> bool:
        """Read the current settings over HTTP when the topic stays silent.

        /motor_settings is published on change, not periodically, so a fresh subscriber waits
        forever on a quiet robot. Measured: the run aborted with "no motor_settings message for
        this joint" although the settings existed in the API.
        """
        url = f"{api_base.rstrip('/')}/api/motor/{self.joint_name}/settings"
        try:
            with urllib.request.urlopen(url, timeout=10) as response:
                data = json.loads(response.read().decode())
        except Exception as error:  # noqa: BLE001 - the caller reports the abort
            _json("baseline_api_failed", url=url, error=str(error))
            return False
        settings = MotorSettings()
        settings.motor_name = data.get("name", self.joint_name)
        settings.turned_on = bool(data.get("turnedOn", True))
        settings.pulse_width_min = int(data.get("pulseWidthMin", 700))
        settings.pulse_width_max = int(data.get("pulseWidthMax", 2500))
        settings.rotation_range_min = int(data.get("rotationRangeMin", -9000))
        settings.rotation_range_max = int(data.get("rotationRangeMax", 9000))
        settings.velocity = int(data.get("velocity", 16000))
        settings.acceleration = int(data.get("acceleration", 10000))
        settings.deceleration = int(data.get("deceleration", 5000))
        settings.period = int(data.get("period", 19500))
        settings.visible = bool(data.get("visible", True))
        settings.upper_endpoint = data.get("upperEndpoint") or "max"
        self._baseline_settings = settings
        _json("baseline_from_api", joint=self.joint_name, velocity=settings.velocity)
        return True

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
        # get_joint_position answers in CENTIDEGREES while /joint_states is radians. Measured on
        # the robot: service 26.0 against joint_states 0.00454 rad for the same head position.
        # Reading it as radians produced a move target 26 rad away, which the service rejected.
        return centideg_to_rad(float(response.position))

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
        # A zero duration is rejected ("apply_joint_trajectory rejected the move"); the move
        # itself is a single point, so the duration only bounds the executor and lets the servo
        # run at its speed setting.
        point.time_from_start.sec = int(self._move_duration_sec)
        point.time_from_start.nanosec = int(
            (self._move_duration_sec % 1.0) * 1_000_000_000
        )
        jt.points = [point]
        request = ApplyJointTrajectory.Request()
        request.joint_trajectory = jt
        future = self._apply_traj.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or not response.successful:
            raise RuntimeError("apply_joint_trajectory rejected the move")

    def restore_baseline_settings(self) -> None:
        """Put the joint back to the settings the run found, whatever happened."""
        base = self._baseline_settings
        if base is None:
            return
        settings = MotorSettings()
        settings.motor_name = base.motor_name
        settings.turned_on = base.turned_on
        settings.pulse_width_min = base.pulse_width_min
        settings.pulse_width_max = base.pulse_width_max
        settings.rotation_range_min = base.rotation_range_min
        settings.rotation_range_max = base.rotation_range_max
        settings.velocity = base.velocity
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
        _json(
            "restored",
            joint=self.joint_name,
            velocity=base.velocity,
            ok=bool(response is not None and response.settings_applied),
        )

    def _range_rad(self) -> tuple[float, float] | None:
        """Joint limits in radians, taken from the baseline settings (stored in centidegrees)."""
        base = self._baseline_settings
        if base is None:
            return None
        low = float(base.rotation_range_min)
        high = float(base.rotation_range_max)
        if low == 0.0 and high == 0.0:
            return None
        return centideg_to_rad(low), centideg_to_rad(high)

    def _home_and_target(self, move_rad: float) -> tuple[float, float]:
        """Pick a target that stays inside the joint range, and keep the home position.

        The first version moved from the last target each run and drifted into the limit: on the
        robot the head walked 0.01 -> 0.51 -> 1.00 -> 1.48 rad and the next move was rejected,
        the index finger sat at its lower limit (-1.569 rad) from the start.
        """
        home = self._read_joint_rad()
        limits = self._range_rad()
        magnitude = abs(move_rad)
        for direction in (1.0, -1.0):
            target = home + direction * magnitude
            if limits is None or limits[0] <= target <= limits[1]:
                return home, target
        raise RuntimeError(
            f"a move of {magnitude:.3f} rad does not fit the range at {home:.3f} rad"
        )

    def _settle_at(self, target_rad: float, timeout_sec: float = 20.0) -> bool:
        """Wait until the joint really is at the target, instead of sleeping a fixed time."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if abs(self._read_joint_rad() - target_rad) <= 0.01:
                return True
            time.sleep(0.1)
        return False

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
        home_rad, target_rad = self._home_and_target(move_rad)
        for run_index in range(runs):
            start_rad = home_rad
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
            if not self._settle_at(start_rad):
                _json(
                    "warn",
                    message="the joint did not return to its home position",
                    home_rad=round(start_rad, 4),
                )
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
    parser.add_argument(
        "--api-base",
        default="http://host.docker.internal",
        help=(
            "Where to read the current motor settings when the motor_settings topic is silent. "
            "localhost does not work from inside the ros-motors container - the Flask API runs "
            "in its own container."
        ),
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=3.0,
        help="Duration given to the single-point move; a zero duration is rejected.",
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
    node = MeasureServoSpeedFloorNode(args.joint, move_duration_sec=args.move_duration)
    violations: list[str] = []
    try:
        if not node._wait_services():
            _json("error", message="ROS services not available")
            return 2
        if not node._wait_baseline_settings():
            # The topic is published on change only, so a quiet robot never sends one.
            if not node._baseline_from_api(args.api_base):
                _json(
                    "error",
                    message=(
                        "no motor_settings message and the API did not answer either — "
                        "start motor_control or pass --api-base"
                    ),
                )
                return 2

        _json(
            "baseline",
            joint=args.joint,
            velocity=node._baseline_settings.velocity,
            acceleration=node._baseline_settings.acceleration,
            source="topic-or-api",
        )

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
        # A crashed run used to leave the joint at the last measured setting: the next
        # invocation then read velocity 1 as its baseline and reported it as such.
        node.restore_baseline_settings()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(run())
