import os
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pib_motors.update_bricklet_uids import *
from pib_motors.motor import motors, Motor

TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", 4223))

class MotorCurrent(Node):

    def __init__(self):

        super().__init__('motor_current')

        self.declare_parameter("frequency", 4.0)
        self.frequency_ = self.get_parameter("frequency").value

        self.motor_current_publisher = self.create_publisher(
            DiagnosticStatus, 
            "motor_current", 
            10)
        
        self.timer = self.create_timer(
            1.0 / self.frequency_, 
            self.publish_motor_current)

        self.get_logger().info("Now Running MOTOR CURRENT")


    def publish_motor_current(self):

        for motor in motors: 
            current = motor.get_current()
            if current == Motor.NO_CURRENT: continue
            self.publish_diagnostic_status(motor.name, current)
        

    def publish_diagnostic_status(self, motor_name: str, current: int) -> None:
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.WARN if current >= 1500 else DiagnosticStatus.OK
        msg.name = motor_name
        keyvalue = KeyValue()
        keyvalue.key = motor_name
        keyvalue.value = str(current)
        msg.values = [keyvalue]
        print(f"publishing: {msg}")
        self.motor_current_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = MotorCurrent()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
