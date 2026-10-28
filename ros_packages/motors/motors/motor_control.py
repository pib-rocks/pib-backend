from typing import Iterable, Mapping, Sequence, Tuple
import math
import rclpy
from datatypes.msg import MotorResult, MotorSettings, SolidStateRelayState
from datatypes.srv import ApplyMotorSettings, ApplyJointTrajectory, GetJointPosition
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
from pib_motors.trajectory_executor import TrajectoryExecutor, is_software_trajectory
from pib_motors.units import centideg_to_rad, rad_to_centideg
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

        if solid_state_relay_bricklet is None:
            self.get_logger().info(
                "No SSR configured. Executing startup pose immediately."
            )
            self._startup_done = True
            self._execute_startup_pose()

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

        if is_legacy_zip_shape(raw):
            outcomes = legacy_zip_outcomes(joint_names)
            response.successful = False
            response.results = motor_outcomes_to_ros(outcomes)
            return response

        validation_errors, validation_outcomes = validate_radians_trajectory(
            raw, name_to_motors, _rad_to_centideg_for_joint
        )
        if validation_errors is not None:
            response.successful = False
            response.results = motor_outcomes_to_ros(validation_outcomes)
            return response

        try:
            jt = joint_trajectory_radians_to_centidegrees(raw, name_to_motors)
            if is_software_trajectory(jt):
                trajectory_ok = self.trajectory_executor.execute(jt)
                if trajectory_ok:
                    outcomes = [(name, True, "", "") for name in joint_names]
                    for motor_name in jt.joint_names:
                        for motor in name_to_motors[motor_name]:
                            self.joint_trajectory_publisher.publish(
                                as_joint_trajectory(motor.name, motor.get_position())
                            )
                else:
                    outcomes = [
                        (name, False, "write_failed", "trajectory execution failed")
                        for name in joint_names
                    ]
                response.successful = outcomes_to_success(outcomes)
                response.results = motor_outcomes_to_ros(outcomes)
                return response

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
            response.position = int(round(motor.get_position()))

        except Exception as e:
            response.successful = False
            response.message = str(e)
            self.get_logger().error(f"error getting position: {str(e)}")
        return response


def main(args=None):

    rclpy.init(args=args)
    motor_control = MotorControl()
    rclpy.spin(motor_control)
    rclpy.shutdown()
    ipcon.disconnect()


if __name__ == "__main__":
    main()
