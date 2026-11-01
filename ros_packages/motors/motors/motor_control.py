from typing import Iterable, Mapping, Sequence, Tuple
import math
import time
from queue import SimpleQueue
import rclpy
from datatypes.msg import MotorResult, MotorSettings, SolidStateRelayState
from datatypes.srv import (
    ApplyMotorSettings,
    ApplyJointTrajectory,
    GetAllJointPositions,
    GetJointPosition,
)
from pib_api_client import motor_client
from pib_motors.bricklet import ipcon, connected_enumerate, solid_state_relay_bricklet
from pib_motors.motor import name_to_motors, motors
from rclpy.node import Node
from pib_motors.startup_pose_executor import StartupPoseExecutor
from pib_motors.motor import Motor
from pib_motors.joint_trajectory_command import (
    MotorOutcome,
    is_legacy_zip_shape,
    iter_single_point_centideg_targets,
    legacy_zip_outcomes,
    outcomes_to_success,
    validate_radians_trajectory,
)
from pib_motors.trajectory_executor import (
    TrajectoryExecutor,
    is_software_trajectory,
    validate_software_trajectory_schedule,
)
from pib_motors.perf_trace import (
    format_perf_trace_joint_state_sweep,
    format_perf_trace_trajectory_done,
    format_perf_trace_trajectory_start,
    log_perf_trace,
)
from pib_motors.bus_writer import (
    all_bus_writers,
    format_bus_writer_summary,
    shutdown_all_bus_writers,
)
from pib_motors.units import centideg_to_rad, rad_to_centideg
from pib_motors.joint_state_sweep import (
    fill_joint_state,
    sweep_feetech_joint_positions,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def motor_settings_ros_to_dto(ms: MotorSettings):
    return {
        "name": ms.motor_name,
        "turnedOn": ms.turned_on,
        "pulseWidthMin": ms.pulse_width_min,
        "pulseWidthMax": ms.pulse_width_max,
        "rotationRangeMin": ms.rotation_range_min,
        "rotationRangeMax": ms.rotation_range_max,
        "velocity": ms.velocity,
        "acceleration": ms.acceleration,
        "deceleration": ms.deceleration,
        "period": ms.period,
        "visible": ms.visible,
        "upperEndpoint": ms.upper_endpoint,
    }


def as_motor_positions(jt: JointTrajectory) -> Iterable[Tuple[str, int]]:
    """Unpack a single-point trajectory into ``(motor_name, centidegree)`` pairs."""
    return iter_single_point_centideg_targets(jt)


def motor_outcomes_to_ros(outcomes: Sequence[MotorOutcome]) -> list[MotorResult]:
    results: list[MotorResult] = []
    for motor_name, successful, error_code, detail in outcomes:
        result = MotorResult()
        result.motor_name = motor_name
        result.successful = successful
        result.error_code = error_code
        result.detail = detail
        results.append(result)
    return results


def as_joint_trajectory(motor_name: str, position_centideg: int) -> JointTrajectory:
    """Build a single-joint ``/joint_trajectory`` message (positions in radians)."""
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    point = JointTrajectoryPoint()
    point.positions.append(centideg_to_rad(position_centideg))
    jt.points = [point]
    return jt


def joint_trajectory_radians_to_centidegrees(
    jt: JointTrajectory,
    name_to_motors: Mapping[str, Sequence[Motor]],
) -> JointTrajectory:
    """Validate ROS radians and return a trajectory for internal centidegree control."""
    converted = JointTrajectory()
    converted.joint_names = list(jt.joint_names)
    names = converted.joint_names
    points = list(jt.points)
    if not names or not points:
        return converted

    for point in points:
        positions = list(point.positions)
        if len(positions) < len(names):
            raise ValueError(
                f"trajectory point has {len(positions)} positions for "
                f"{len(names)} joint names"
            )
        new_point = JointTrajectoryPoint()
        new_point.time_from_start = point.time_from_start
        new_point.velocities = list(point.velocities)
        new_point.accelerations = list(point.accelerations)
        new_point.effort = list(point.effort)
        for joint_name, rad in zip(names, positions):
            new_point.positions.append(
                _rad_to_centideg_for_joint(joint_name, float(rad), name_to_motors)
            )
        converted.points.append(new_point)

    return converted


def _rad_to_centideg_for_joint(
    joint_name: str,
    radians: float,
    name_to_motors: Mapping[str, Sequence[Motor]],
) -> float:
    if joint_name not in name_to_motors:
        raise ValueError(f"unknown joint name '{joint_name}'")
    if not math.isfinite(radians):
        raise ValueError(f"joint '{joint_name}' position must be finite, got {radians}")
    centideg = rad_to_centideg(radians)
    motor = name_to_motors[joint_name][0]
    min_pos = motor.rotation_range_min
    max_pos = motor.rotation_range_max
    if centideg < min_pos or centideg > max_pos:
        raise ValueError(
            f"joint '{joint_name}' position {radians} rad ({centideg} centideg) "
            f"is outside allowed range [{min_pos}, {max_pos}] centidegrees"
        )
    return float(centideg)


class MotorControl(Node):

    def __init__(self):

        super().__init__("motor_control")

        # Toggle Devmode
        self.declare_parameter("dev", False)
        self.dev = self.get_parameter("dev").value

        self.declare_parameter("joint_state_publish_rate_hz", 10.0)
        rate_hz = float(self.get_parameter("joint_state_publish_rate_hz").value)
        if rate_hz <= 0:
            rate_hz = 10.0

        # Service for JointTrajectory
        self.srv = self.create_service(
            ApplyJointTrajectory, "apply_joint_trajectory", self.apply_joint_trajectory
        )

        # Publisher for JointTrajectory
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "joint_trajectory", 10
        )

        # Service for MotorSettings
        self.srv = self.create_service(
            ApplyMotorSettings, "apply_motor_settings", self.apply_motor_settings
        )

        # Service for Getting Joint Position
        self.srv_get_position = self.create_service(
            GetJointPosition, "get_joint_position", self.get_joint_position
        )

        self.srv_get_all_positions = self.create_service(
            GetAllJointPositions,
            "get_all_joint_positions",
            self.get_all_joint_positions,
        )

        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self._joint_state_timer = self.create_timer(
            1.0 / rate_hz, self._publish_joint_states
        )
        self._trajectory_completion_queue: SimpleQueue = SimpleQueue()
        self._trajectory_completion_timer = self.create_timer(
            0.05, self._process_trajectory_completions
        )
        self._bus_writer_stats_timer = self.create_timer(
            1.0, self._log_bus_writer_stats
        )

        # Publisher for MotorSettings
        self.motor_settings_publisher = self.create_publisher(
            MotorSettings, "motor_settings", 10
        )

        self._startup_done = False

        # load motor-settings if not in dev mode
        if not self.dev:
            for motor in motors:
                if motor.check_if_motor_is_connected():
                    successful, motor_settings_dto = motor_client.get_motor_settings(
                        motor.name
                    )
                    if successful:
                        if not self._startup_done:
                            motor_settings_dto["turnedOn"] = False
                        motor.apply_settings(motor_settings_dto)

        # Log that initialization is complete
        self.get_logger().info("Now Running MOTOR_CONTROL")

        # Register and trigger enumeration of available bricklets to detect connected devices
        ipcon.register_callback(ipcon.CALLBACK_ENUMERATE, connected_enumerate)
        ipcon.enumerate()

        self.ssr_subscriber = self.create_subscription(
            SolidStateRelayState,
            "solid_state_relay_state",
            self.on_ssr_state_change,
            10,
        )

        self.startup_pose_executor = StartupPoseExecutor(self, motors=motors)
        self.trajectory_executor = TrajectoryExecutor(self, name_to_motors)

        #: Last known motor power state: None until a source told us, then True/False.
        self._relay_on: bool | None = None
        if solid_state_relay_bricklet is None:
            self.get_logger().info(
                "No SSR configured. Executing startup pose immediately."
            )
            self._startup_done = True
            self._execute_startup_pose()
        else:
            self._read_relay_state()

    def _read_relay_state(self) -> None:
        """Ask the relay brick directly once, so the first tick is not blind.

        The topic only fires on a change; without this the node would assume
        nothing and pay 26 serial timeouts per sweep while the power is off.
        """
        if solid_state_relay_bricklet is None:
            return
        try:
            self._relay_on = bool(solid_state_relay_bricklet.get_state())
        except Exception as error:  # noqa: BLE001 - unknown is a valid state
            self.get_logger().warn(f"Could not read the relay state: {error}")
            self._relay_on = None

    def _motor_power_is_off(self) -> bool:
        """True only when the relay definitely reports off.

        Unknown counts as on: a build without a relay, or a relay that has not
        answered yet, must not silently stop publishing joint states.
        """
        return self._relay_on is False

    def _execute_startup_pose(self):
        try:
            success = self.startup_pose_executor.execute()
            if success:
                self.get_logger().info("Startup pose execution completed successfully.")
            else:
                self.get_logger().warn(
                    "Startup pose execution completed with warnings."
                )
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error while applying startup pose: {str(e)}"
            )

    def on_ssr_state_change(self, msg: SolidStateRelayState):
        self._relay_on = bool(msg.turned_on)
        if not msg.turned_on or self._startup_done:
            return
        self._startup_done = True
        self._execute_startup_pose()

    def apply_motor_settings(
        self, request: ApplyMotorSettings.Request, response: ApplyMotorSettings.Response
    ) -> ApplyMotorSettings.Response:

        response.settings_applied = True
        response.settings_persisted = True

        motor_settings_ros = request.motor_settings
        motor_settings_dto = motor_settings_ros_to_dto(motor_settings_ros)

        try:
            motors = name_to_motors[request.motor_settings.motor_name]
            if (
                motor_settings_ros.pulse_width_min == -1
                and motor_settings_ros.pulse_width_max == -1
            ):
                for motor in motors:
                    applied = motor.calibrate_zero()
                    response.settings_applied &= applied
                    if applied:
                        self.joint_trajectory_publisher.publish(
                            as_joint_trajectory(motor.name, 0)
                        )
                        stored = motor.get_settings()
                        motor_settings_dto["pulseWidthMin"] = stored["pulseWidthMin"]
                        motor_settings_dto["pulseWidthMax"] = stored["pulseWidthMax"]
                        motor_settings_dto["turnedOn"] = True
                        motor_settings_ros.pulse_width_min = stored["pulseWidthMin"]
                        motor_settings_ros.pulse_width_max = stored["pulseWidthMax"]
                        motor_settings_ros.turned_on = True
                        if applied or self.dev:
                            persisted, _ = motor_client.update_motor_settings(
                                motor.name, motor_settings_dto
                            )
                            response.settings_persisted &= persisted
                            self.motor_settings_publisher.publish(motor_settings_ros)
                        self.get_logger().info(
                            f"calibrated zero on motor: {str(motor)}"
                        )
                return response

            for motor in motors:
                motor_settings_dto["name"] = motor.name
                motor_settings_ros.motor_name = motor.name
                applied = motor.apply_settings(motor_settings_dto)
                response.settings_applied &= applied
                if applied or self.dev:
                    persisted, _ = motor_client.update_motor_settings(
                        motor.name, motor_settings_dto
                    )
                    response.settings_persisted &= persisted
                    self.motor_settings_publisher.publish(motor_settings_ros)
                self.get_logger().info(f"updated motor: {str(motor)}")

        except Exception as e:
            response.settings_applied = False
            response.settings_persisted = False
            self.get_logger().warn(
                f"Error while processing motor-settings-message: {str(e)}"
            )

        return response

    def _process_trajectory_completions(self) -> None:
        while not self._trajectory_completion_queue.empty():
            completion = self._trajectory_completion_queue.get()
            log_perf_trace(
                self.get_logger(),
                format_perf_trace_trajectory_done(
                    completion.duration_sec,
                    completion.successful,
                    completion.cancelled,
                ),
            )
            if completion.successful and not completion.cancelled:
                for motor_name in completion.joint_names:
                    for motor in name_to_motors[motor_name]:
                        self.joint_trajectory_publisher.publish(
                            as_joint_trajectory(motor.name, motor.get_position())
                        )

    def _stop_running_software_trajectory(self) -> None:
        if self.trajectory_executor.is_running():
            self.trajectory_executor.cancel()

    def apply_joint_trajectory(
        self,
        request: ApplyJointTrajectory.Request,
        response: ApplyJointTrajectory.Response,
    ) -> ApplyJointTrajectory.Response:
        """Apply a ``trajectory_msgs/JointTrajectory`` whose positions are in radians.

        Positions are converted once at entry to integer centidegrees for motors and
        ``TrajectoryExecutor``. The full command is validated before any motor write.
        """
        raw = request.joint_trajectory
        joint_names = list(raw.joint_names)
        response.results = []
        trace_started = time.monotonic()
        point_count = len(raw.points)

        if not joint_names and not raw.points:
            if self.trajectory_executor.is_running():
                self._stop_running_software_trajectory()
                response.successful = True
                response.results = []
            else:
                response.successful = False
                response.results = motor_outcomes_to_ros(
                    [
                        (
                            "",
                            False,
                            "invalid_trajectory",
                            "empty trajectory does not cancel a running sequence",
                        )
                    ]
                )
            return response

        if is_legacy_zip_shape(raw):
            outcomes = legacy_zip_outcomes(joint_names)
            response.successful = False
            response.results = motor_outcomes_to_ros(outcomes)
            self.get_logger().warning(
                f"apply_joint_trajectory rejected legacy zip shape for {len(joint_names)} joints"
            )
            return response

        validation_errors, validation_outcomes = validate_radians_trajectory(
            raw, name_to_motors, _rad_to_centideg_for_joint
        )
        if validation_errors is not None:
            response.successful = False
            response.results = motor_outcomes_to_ros(validation_outcomes)
            self.get_logger().warning(
                f"apply_joint_trajectory rejected invalid trajectory: {validation_errors}"
            )
            return response

        if is_software_trajectory(raw):
            schedule_error = validate_software_trajectory_schedule(raw)
            if schedule_error is not None:
                response.successful = False
                response.results = motor_outcomes_to_ros(
                    [
                        (name, False, "invalid_schedule", schedule_error)
                        for name in joint_names
                    ]
                )
                self.get_logger().warning(
                    f"apply_joint_trajectory rejected invalid schedule: {schedule_error}"
                )
                return response

        try:
            jt = joint_trajectory_radians_to_centidegrees(raw, name_to_motors)
            self._emit_trajectory_start(point_count, trace_started)
            if is_software_trajectory(jt):
                if self.trajectory_executor.is_running():
                    self._stop_running_software_trajectory()
                started, start_error = self.trajectory_executor.start_async(
                    jt, self._trajectory_completion_queue
                )
                if not started:
                    detail = start_error or "failed to start software trajectory"
                    outcomes = [
                        (name, False, "trajectory_rejected", detail)
                        for name in joint_names
                    ]
                    response.successful = False
                    response.results = motor_outcomes_to_ros(outcomes)
                    return response
                outcomes = [(name, True, "", "") for name in joint_names]
                response.successful = outcomes_to_success(outcomes)
                response.results = motor_outcomes_to_ros(outcomes)
                return response

            if self.trajectory_executor.is_running():
                self._stop_running_software_trajectory()

            apply_outcomes: list[MotorOutcome] = []
            for motor_name, position in as_motor_positions(jt):
                motor_ok = True
                detail = ""
                for motor in name_to_motors[motor_name]:
                    self.get_logger().info(
                        f"setting position of {motor.name} to {position}"
                    )
                    successful = motor.set_position(position)
                    self.get_logger().info(
                        f"setting position {'succeeded' if successful else 'failed'}."
                    )
                    motor_ok &= successful
                    if successful:
                        self.joint_trajectory_publisher.publish(
                            as_joint_trajectory(motor.name, position)
                        )
                    else:
                        detail = f"failed to write position for motor '{motor.name}'"
                if motor_ok:
                    apply_outcomes.append((motor_name, True, "", ""))
                else:
                    apply_outcomes.append((motor_name, False, "write_failed", detail))
            response.successful = outcomes_to_success(apply_outcomes)
            response.results = motor_outcomes_to_ros(apply_outcomes)
        except Exception as e:
            response.successful = False
            response.results = motor_outcomes_to_ros(
                [(name, False, "out_of_range", str(e)) for name in joint_names]
            )
            self.get_logger().error(f"error while applying joint-trajectory: {str(e)}")
        return response

    def _emit_trajectory_start(
        self, point_count: int, started_monotonic: float
    ) -> None:
        elapsed_ms = (time.monotonic() - started_monotonic) * 1000.0
        log_perf_trace(
            self.get_logger(),
            format_perf_trace_trajectory_start(point_count, elapsed_ms),
        )

    def _publish_joint_states(self) -> None:
        if self._motor_power_is_off():
            # No power on the bus: every read would burn its serial timeout.
            self._relay_off_skips = getattr(self, "_relay_off_skips", 0) + 1
            return
        started = time.monotonic()
        sweep = sweep_feetech_joint_positions(motors)
        positions = sweep.positions_centideg
        if not positions:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        fill_joint_state(msg, positions)
        self.joint_state_publisher.publish(msg)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        log_perf_trace(
            self.get_logger(),
            format_perf_trace_joint_state_sweep(
                len(positions),
                elapsed_ms,
                sweep.ports_succeeded,
                sweep.ports_attempted,
            ),
        )

    def _log_bus_writer_stats(self) -> None:
        for writer in all_bus_writers():
            log_perf_trace(self.get_logger(), format_bus_writer_summary(writer))

    def get_all_joint_positions(
        self,
        request: GetAllJointPositions.Request,
        response: GetAllJointPositions.Response,
    ) -> GetAllJointPositions.Response:
        filter_names = [name for name in request.joint_names if name]
        if filter_names:
            target_names = filter_names
        else:
            target_names = sorted(name_to_motors.keys())

        response.header = Header()
        response.header.stamp = self.get_clock().now().to_msg()
        response.results = []
        response.positions = []

        if self._motor_power_is_off():
            response.successful = False
            response.message = "motor power is off (solid state relay off)"
            for joint_name in target_names:
                result = MotorResult()
                result.motor_name = joint_name
                result.successful = False
                if joint_name not in name_to_motors:
                    result.error_code = "unknown_joint"
                    result.detail = f"unknown joint name '{joint_name}'"
                else:
                    result.error_code = "bus_unavailable"
                    result.detail = response.message
                response.results.append(result)
                response.positions.append(0)
            return response

        sweep = sweep_feetech_joint_positions(motors)
        if (
            sweep.ports_attempted == 0
            and not sweep.ports_unreachable
            and not sweep.positions_centideg
        ):
            response.successful = False
            response.message = "no sweepable actuator family on this robot (servo bricklets report no position feedback)"
            for joint_name in target_names:
                result = MotorResult()
                result.motor_name = joint_name
                if joint_name not in name_to_motors:
                    result.successful = False
                    result.error_code = "unknown_joint"
                    result.detail = f"unknown joint name '{joint_name}'"
                else:
                    result.successful = False
                    result.error_code = "bus_unavailable"
                    result.detail = response.message
                response.results.append(result)
                response.positions.append(0)
            return response

        bus_error = ""
        if not sweep.positions_centideg and sweep.ports_unreachable:
            bus_error = (
                "no Feetech bus reachable "
                f"({sweep.ports_unreachable} port(s) did not answer; "
                "adapter absent or motor power off)"
            )
        response.successful = not bus_error
        response.message = bus_error
        for joint_name in target_names:
            result = MotorResult()
            result.motor_name = joint_name
            if joint_name not in name_to_motors:
                result.successful = False
                result.error_code = "unknown_joint"
                result.detail = f"unknown joint name '{joint_name}'"
                response.positions.append(0)
            elif joint_name in sweep.positions_centideg:
                result.successful = True
                result.error_code = ""
                result.detail = ""
                response.positions.append(int(sweep.positions_centideg[joint_name]))
            elif bus_error:
                result.successful = False
                result.error_code = "bus_unavailable"
                result.detail = bus_error
                response.positions.append(0)
            else:
                result.successful = False
                result.error_code = "position_unavailable"
                result.detail = "no measurement in bus sweep"
                response.positions.append(0)
            response.results.append(result)
        return response

    def get_joint_position(
        self,
        request: GetJointPosition.Request,
        response: GetJointPosition.Response,
    ) -> GetJointPosition.Response:
        joint_name = request.joint_name
        response.successful = True
        try:
            if joint_name not in name_to_motors:
                response.successful = False
                response.message = f"unknown joint name '{joint_name}'"
                return response

            motors_for_joint = name_to_motors[joint_name]
            motor = motors_for_joint[0]
            response.position = int(round(motor.get_current_position()))

        except Exception as e:
            response.successful = False
            response.message = str(e)
            self.get_logger().error(f"error getting position: {str(e)}")
        return response


def main(args=None):

    rclpy.init(args=args)
    motor_control = MotorControl()
    try:
        rclpy.spin(motor_control)
    finally:
        shutdown_all_bus_writers()
        motor_control.destroy_node()
    rclpy.shutdown()
    ipcon.disconnect()


if __name__ == "__main__":
    main()
