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
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue



class Motor_current(Node):

    def __init__(self):

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        super().__init__('motor_current')
        self.declare_parameter("frequency", 4.0)
        self.frequency_ = self.get_parameter("frequency").value
        self.current_publisher_ = self.create_publisher(DiagnosticStatus, "motor_current", 10)
        self.publish_ = self.create_timer(1.0/self.frequency_, self.callback_publisher)
        self.get_logger().info(self.get_name() + ": self.__init__ passed")
        self.current_values_servo1_ = [0,0,0,0,0,0,0,0,0,0]
        self.current_values_servo2_ = [0,0,0,0,0,0,0,0,0,0]
        self.current_values_servo3_ = [0,0,0,0,0,0,0]
        self.active_servos_ = [True,True,True]
        self.motors_servo1_ = [
            "turn_head_motor",
            "tilt_forward_motor",
            "wrist_left",
            "thumb_right_opposition",
            "thumb_right_stretch",
            "index_right_stretch",
            "middle_right_stretch",
            "ring_right_stretch",
            "pinky_right_stretch",
            "thumb_left_opposition",
        ]
        self.motors_servo2_ = [
            "thumb_left_stretch",
            "index_left_stretch",
            "middle_left_stretch",
            "ring_left_stretch",
            "pinky_left_stretch",
            "upper_arm_left_rotation",
            "elbow_left",
            "lower_arm_left_rotation",
            "tilt_sideways_motor",
        ]
        self.motors_servo3_ = [
            "shoulder_horizontal_left",
            "upper_arm_right_rotation",
            "elbow_right",
            "lower_arm_right_rotation",
            "wrist_right",
            "shoulder_vertical_right",
            "shoulder_horizontal_right",
            "shoulder_vertical_left",
        ]
        #servo to bricklet map, All motor names for each servo bricklet and map it to bricklet 1, 2, 3
        try:
            self.ipcon = IPConnection()  # Create IP connection
            self.hat = BrickHAT("X", self.ipcon)
            # Handles for three Servo Bricklets
            self.servo1 = BrickletServoV2(UID1, self.ipcon)
            self.servo2 = BrickletServoV2(UID2, self.ipcon)
            self.servo3 = BrickletServoV2(UID3, self.ipcon)
            self.ipcon.connect(HOST, 4223)
            self.get_logger().info(self.get_name() + ": servo init complete")
        except Exception as e:
            self.get_logger().warn(f"Error servo init: {str(e)}")

    def callback_publisher(self):
        self.iterate_servos(self.servo1, self.current_values_servo1_, self.motors_servo1_, 0)
        self.iterate_servos(self.servo2, self.current_values_servo2_, self.motors_servo2_, 1)
        self.iterate_servos(self.servo3, self.current_values_servo3_, self.motors_servo3_, 2)
        #self.publish_diagnostic(1000, 3, self.motors_servo1_) #Testmessage
    def iterate_servos(self, servo, current_values, motors_servo, active_servo):
        if self.active_servos_[active_servo] :
            try:
                for x in range(len(motors_servo)-1):
                    current = servo.get_servo_current(x)
                    if current != current_values[x]:
                        self.publish_diagnostic(current, x, motors_servo)
            except Exception as e:
                self.get_logger().warn(f"Error on Servo: {str(e)}")
                self.active_servos_[active_servo] = False
                self.get_logger().warn("ActiveServos: " + str(self.active_servos_))
		
        
    def publish_diagnostic(self, intvalue, index, motors_servo):
        msg = DiagnosticStatus()
        if intvalue >= 1500 :
            msg.level = DiagnosticStatus.WARN
        else:
            msg.level = DiagnosticStatus.OK
        msg.name = motors_servo[index]
        msg.message = ""
        msg.hardware_id = ""
        keyvalue = KeyValue()
        keyvalue.key = motors_servo[index]
        keyvalue.value = str(intvalue)
        msg.values = [keyvalue]
        self.current_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    motor_current = Motor_current()
    rclpy.spin(motor_current)
    rclpy.shutdown()
    motor_current.ipcon.disconnect()


if __name__ == '__main__':
    main()
