HOST = "localhost"
PORT = 4223
UID1 = "XYZ" # Replace with the UID of first Servo Bricklet
UID2 = "XYZ" # Replace with the UID of second Servo Bricklet
UID3 = "XYZ" # Replace with the UID of third Servo Bricklet

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class Motor_control(Node):

    def __init__(self):

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',  # Topic name that Cerebra publishes to
            self.callback,
            qos_profile = qos_policy
        )

        self.get_logger().warn(f"Info: passed __init__")

        #servo to bricklet map, All motor names for each servo bricklet and map it to bricklet 1, 2, 3
        self.motor_bricklet_map = {
            "turn_head_motor": 1,
            "tilt_forward_motor": 1,
            "tilt_sideways_motor": 1,
            "thumb_left_opposition": 1,
            "thumb_left_stretch": 1,
            "index_left_stretch": 1,
            "middle_left_stretch": 1,
            "ring_left_stretch": 1,
            "pinky_left_stretch": 1,
            "thumb_right_opposition": 1,
            "thumb_right_stretch": 2,
            "index_right_stretch": 2,
            "middle_right_stretch": 2,
            "ring_right_stretch": 2,
            "pinky_right_stretch": 2,
            "/upper_arm_left_rotation": 2,
            "/elbow_left": 2,
            "/lower_arm_left_rotation": 2,
            "/wrist_left": 2,
            "/shoulder_vertical_left": 2,
            "/shoulder_horizontal_left": 3,
            "/upper_arm_right_rotation": 3,
            "/elbow_right": 3,
            "/lower_arm_right_rotation": 3,
            "/wrist_right": 3,
            "/shoulder_vertical_right": 3,
            "/shoulder_horizontal_right": 3,
        }
        #Servo to port/pin map
        self.motor_map = {
            "turn_head_motor": 0,
            "tilt_forward_motor": 1,
            "tilt_sideways_motor": 2,
            "thumb_left_opposition": 3,
            "thumb_left_stretch": 4,
            "index_left_stretch": 5,
            "middle_left_stretch": 6,
            "ring_left_stretch": 7,
            "pinky_left_stretch": 8,
            "thumb_right_opposition": 9,
            "thumb_right_stretch": 0,
            "index_right_stretch": 1,
            "middle_right_stretch": 2,
            "ring_right_stretch": 3,
            "pinky_right_stretch": 4,
            "/upper_arm_left_rotation": 5,
            "/elbow_left": 6,
            "/lower_arm_left_rotation": 7,
            "/wrist_left": 8,
            "/shoulder_horizontal_left": 0,
            "/upper_arm_right_rotation": 1,
            "/elbow_right": 2,
            "/lower_arm_right_rotation": 3,
            "/wrist_right": 4,
            "/shoulder_horizontal_right": 6,
            #shoulder vertical left and right are added at the end in an if condition (they are using two motors instead of one)
            #shoulder_vertical_right is connected to bricklet 3 pins 5 & 8 | shoulder_vertical_left is connected to bricklet 3 pins 7 & 9
        }
        self.ipcon = IPConnection()  # Create IP connection
        self.hat = BrickHAT(UID, self.ipcon)
        # Handles for three Servo Bricklets
        self.servo1 = BrickletServoV2(UID1, self.ipcon)
        self.servo2 = BrickletServoV2(UID2, self.ipcon)
        self.servo3 = BrickletServoV2(UID3, self.ipcon)
        self.ipcon.connect(HOST, 4223)

    def callback(self, msg):
        try:
            if len(msg.joint_names) == 0:
                raise Exception("Sorry, no numbers below zero")
                self.get_logger().warn(f"Error processing message: {str(e)}")
            else:
                for arrayNumber in range(len(msg.joint_names)):
                    # Extract the message sent by cerebra, isolate motor and value raw value
                    motor = str(msg.joint_names[arrayNumber])
                    self.get_logger().info(f"Motor: {motor}")
                    value = msg.points[arrayNumber].positions[0]
                    self.get_logger().info(f"Value: {value}")
                    motor_port = self.motor_map.get(motor)
                    self.get_logger().info(f"Motor_Port: {motor_port}")
                    motor_bricklet = self.motor_bricklet_map.get(motor)
                    self.get_logger().info(f"Motor_Bricklet: {motor_bricklet}")

                    # Move motor retrieved from message, each condition depends on which bricklet motor is connected to
                    if motor_bricklet == 1: #Improve range of motion by modifying pwm, then move motor 
                        self.servo1.set_pulse_width(motor_port, 700, 2500)
                        self.servo1.set_position(motor_port, value)
                        self.servo1.set_motion_configuration(motor_port, 9000, 9000, 7000)
                        self.servo1.set_enable(motor_port, True)
                    elif motor_bricklet == 2:
                        self.servo2.set_position(motor_port, value)
                        self.servo2.set_pulse_width(motor_port, 700, 2500)
                        self.servo2.set_motion_configuration(motor_port, 9000, 9000, 7000)
                        self.servo2.set_enable(motor_port, True)            
                    elif motor_bricklet == 3:
                        if motor == "/shoulder_vertical_right":
                            self.servo3.set_position(5, value)
                            self.servo3.set_position(8, value)
                            self.servo3.set_pulse_width(5, 700, 2500)
                            self.servo3.set_pulse_width(8, 700, 2500)
                            self.servo3.set_motion_configuration(5, 9000, 9000, 7000)
                            self.servo3.set_motion_configuration(8, 9000, 9000, 7000)
                            self.servo3.set_enable(5, True)
                            self.servo3.set_enable(8, True)
                        elif motor == "/shoulder_vertical_left": 	
                            self.servo3.set_position(7, value)
                            self.servo3.set_position(9, value)
                            self.servo3.set_pulse_width(7, 700, 2500)
                            self.servo3.set_pulse_width(9, 700, 2500)
                            self.servo3.set_motion_configuration(7, 9000, 9000, 7000)
                            self.servo3.set_motion_configuration(9, 9000, 9000, 7000)
                            self.servo3.set_enable(7, True)
                            self.servo3.set_enable(9, True)
                        else:
                            self.servo3.set_position(motor_port, value)
                            self.servo3.set_pulse_width(motor_port, 700, 2500)
                            self.servo3.set_motion_configuration(motor_port, 9000, 9000, 7000)
                            self.servo3.set_enable(motor_port, True)

        except Exception as e:
            self.get_logger().warn(f"Error processing message: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    motor_control = Motor_control()
    rclpy.spin(motor_control)
    rclpy.shutdown()
    motor_control.ipcon.disconnect()


if __name__ == '__main__':
    main()
