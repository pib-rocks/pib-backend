import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from datatypes.srv import MotorSettingsSrv
from datatypes.msg import MotorSettings
from pib_api_client import motor_client
from pib_motors.motor import name_to_motors, motors
from pib_motors.bricklet import ipcon
import sys
sys.path.append('/home/pib/ros_working_dir/src/motors/utils')
import update_bricklet_uids

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
        "invert": ms.invert
    }


class MotorControl(Node):

    def __init__(self):

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        super().__init__('motor_control')

        # Toggle Devmode
        self.declare_parameter("dev", False)
        self.dev = self.get_parameter("dev").value

        # Topic for JointTrajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.joint_trajectory_callback,
            qos_profile=qos_policy
        )

        # Service for MotorSettings
        self.srv = self.create_service(
            MotorSettingsSrv,
            'motor_settings',
            self.motor_settings_callback
        )

        # Publisher for MotorSettings
        self.publisher = self.create_publisher(
            MotorSettings, "motor_settings", 10)

        # load motor-settings if not in dev mode
        if not self.dev:
            for motor in motors:
                if self.check_if_motor_is_connected(motor):
                    successful, motor_settings_dto = motor_client.get_motor_settings(
                        motor.name)
                    if successful:
                        motor.apply_settings(motor_settings_dto)

        # get UID from database
        response = update_bricklet_uids.get_uids_from_db()
        UID1 = response[0]
        UID2 = response[1]
        UID3 = response[2]

        # Log that initialization is complete
        self.get_logger().warn("Info: passed __init__")

    # Check if motor is connected
    # Non-connected bricklet UIDs seem to be only 2 characters long
    # Connected ones at least 6 characters long
    def check_if_motor_is_connected(self, motor):
        for bricklet_pin in motor.bricklet_pins:
            if len(str(bricklet_pin.bricklet.uid)) >= 6:
                return True
        return False

    def motor_settings_callback(self, request: MotorSettingsSrv.Request, response: MotorSettingsSrv.Response):

        response.settings_applied = True
        response.settings_persisted = True

        motor_settings_ros = request.motor_settings
        motor_settings_dto = motor_settings_ros_to_dto(motor_settings_ros)

        try:
            motors = name_to_motors[request.motor_settings.motor_name]
            for motor in motors:
                motor_settings_dto['name'] = motor.name
                motor_settings_ros.motor_name = motor.name
                applied = motor.apply_settings(motor_settings_dto)
                response.settings_applied &= applied
                if applied or self.dev:
                    persisted, _ = motor_client.update_motor_settings(
                        motor.name, motor_settings_dto)
                    response.settings_persisted &= persisted
                    self.publisher.publish(motor_settings_ros)
                self.get_logger().info(f'updated motor: {str(motor)}')

        except Exception as e:
            response = MotorSettingsSrv.Response(
                settings_applied=False, settings_persisted=False)
            self.get_logger().warn(
                f"Error while processing motor-settings-message: {str(e)}")

        return response

    def joint_trajectory_callback(self, joint_trajectory: JointTrajectory):

        try:
            motor_name = joint_trajectory.joint_names[0]
            target_position = joint_trajectory.points[0].positions[0]

            for motor in name_to_motors[motor_name]:
                if self.check_if_motor_is_connected(motor):
                    self.get_logger().info(f"setting position of '{motor.name}' to {target_position}.")
                    motor.set_position(target_position)
                    self.get_logger().info(f"position of '{motor.name}' was set to {motor.get_position()}.")
                else:
                    self.get_logger().info(f"Motor is not connected.")

        except Exception as e:
            self.get_logger().warn(
                f"Error while processing joint-trajectory-message: {str(e)}")


def main(args=None):

    rclpy.init(args=args)
    motor_control = MotorControl()
    rclpy.spin(motor_control)
    rclpy.shutdown()
    ipcon.disconnect()


if __name__ == '__main__':
    main()
