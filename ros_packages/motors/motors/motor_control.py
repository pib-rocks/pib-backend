from typing import Iterable, Tuple

import rclpy
from datatypes.msg import MotorSettings
from datatypes.srv import ApplyMotorSettings, ApplyJointTrajectory, GetJointPosition
from pib_api_client import motor_client
from pib_motors.bricklet import ipcon, connected_enumerate
from pib_motors.motor import name_to_motors, motors
from rclpy.node import Node
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
        "invert": ms.invert,
    }


def as_motor_positions(jt: JointTrajectory) -> Iterable[Tuple[str, int]]:
    """unpacks a jt-message into an iterable of motorname-position-pairs"""
    motor_names = jt.joint_names
    points: list[JointTrajectoryPoint] = jt.points
    positions = (point.positions[0] for point in points)
    return zip(motor_names, positions)


def as_joint_trajectory(motor_name: str, position: int) -> JointTrajectory:
    """converts a motorname and position into a simple jt-message"""
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    point = JointTrajectoryPoint()
    point.positions.append(position)
    jt.points = [point]
    return jt


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

        # load motor-settings if not in dev mode
        if not self.dev:
            for motor in motors:
                if motor.check_if_motor_is_connected():
                    successful, motor_settings_dto = motor_client.get_motor_settings(
                        motor.name
                    )
                    if successful:
                        motor.apply_settings(motor_settings_dto)

        # Log that initialization is complete
        self.get_logger().info("Now Running MOTOR_CONTROL")

        # Register and trigger enumeration of available bricklets to detect connected devices
        ipcon.register_callback(ipcon.CALLBACK_ENUMERATE, connected_enumerate)
        ipcon.enumerate()

    def apply_motor_settings(
        self, request: ApplyMotorSettings.Request, response: ApplyMotorSettings.Response
    ) -> ApplyMotorSettings.Response:

        response.settings_applied = True
        response.settings_persisted = True

        motor_settings_ros = request.motor_settings
        motor_settings_dto = motor_settings_ros_to_dto(motor_settings_ros)

        try:
            motors = name_to_motors[request.motor_settings.motor_name]
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
        jt = request.joint_trajectory
        response.successful = True
        try:
            for motor_name, position in as_motor_positions(jt):
                for motor in name_to_motors[motor_name]:
                    self.get_logger().info(
                        f"setting position of {motor.name} to {position}"
                    )
                    successful = motor.set_position(position)
                    self.get_logger().info(
                        f"setting position {'succeeded' if successful else 'failed'}."
                    )
                    response.successful &= successful
                    self.joint_trajectory_publisher.publish(
                        as_joint_trajectory(motor.name, position)
                    )
        except Exception as e:
            response.successful = False
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
            if joint_name in name_to_motors:
                motors_for_joint = name_to_motors[joint_name]
                if motors_for_joint:
                    motor = motors_for_joint[0]
                    response.position = float(motor.get_position())
                else:
                    response.successful = False
                    response.message = f"no motors found for '{joint_name}'"
            else:
                response.successful = False
                response.message = f"unknown joint name '{joint_name}'"
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
