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
from datatypes.srv import MotorSettingsSrv


class Motor:	

		def __init__(self, name, servo, ports):
			self.name = name
			self.servo = servo
			self.ports = ports

		def __str__(self):
			return f"(name={self.name}, servo={self.servo.uid}, ports={self.ports})"

		# Only for testing -> may be removed	
		def get_settings(self, msg, port):
			msg.motor_name = self.name
			msg.pulse_width_min, msg.pulse_width_max = [float(v) for v in self.servo.get_pulse_width(port)]
			msg.rotation_range_min, msg.rotation_range_max = [float(v) for v in self.servo.get_degree(port)]
			msg.velocity, msg.acceleration, msg.deceleration = [float(v) for v in self.servo.get_motion_configuration(port)]
			msg.period = float(self.servo.get_period(port))
			msg.pulse_width_min, msg.pulse_width_max = [float(v) for v in self.servo.get_pulse_width(port)]
			msg.turned_on = self.servo.get_enabled(port)


class Motor_control(Node):

	def __init__(self):

		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

		super().__init__('motor_control')

		# Topic for JointTrajectory
		self.subscription = self.create_subscription(
			JointTrajectory,
			'joint_trajectory',  # Topic name that Cerebra publishes to
			self.joint_trajectory_callback,
			qos_profile = qos_policy
		)

		# Service for MotorSettings
		self.srv = self.create_service(
			MotorSettingsSrv, 
			'motor_settings', # Service name that Cerebra calls
			self.motor_settings_callback
		)

		# Connection
		self.ipcon = IPConnection()  # Create IP connection
		self.hat = BrickHAT("X", self.ipcon)
		self.ipcon.connect(HOST, 4223)

		# Handles for three Servo Bricklets
		self.servo1 = BrickletServoV2(UID1, self.ipcon)
		self.servo2 = BrickletServoV2(UID2, self.ipcon)
		self.servo3 = BrickletServoV2(UID3, self.ipcon)

		# Available motors
		self.motors = [
			Motor("turn_head_motor", self.servo1, [0]),
			Motor("tilt_forward_motor", self.servo1, [1]),
			Motor("tilt_sideways_motor", self.servo1, [2]),
			Motor("thumb_left_opposition", self.servo1, [3]),
			Motor("thumb_left_stretch", self.servo1, [4]),
			Motor("index_left_stretch", self.servo1, [5]),
			Motor("middle_left_stretch", self.servo1, [6]),
			Motor("ring_left_stretch", self.servo1, [7]),
			Motor("pinky_left_stretch", self.servo1, [8]),
			Motor("thumb_right_opposition", self.servo1, [9]),
			Motor("thumb_right_stretch", self.servo2, [0]),
			Motor("index_right_stretch", self.servo2, [1]),
			Motor("middle_right_stretch", self.servo2, [2]),
			Motor("ring_right_stretch", self.servo2, [3]),
			Motor("pinky_right_stretch", self.servo2, [4]),
			Motor("/upper_arm_left_rotation", self.servo2, [5]),
			Motor("/elbow_left", self.servo2, [6]),
			Motor("/lower_arm_left_rotation", self.servo2, [7]),
			Motor("/wrist_left", self.servo2, [8]),
			Motor("/shoulder_vertical_left", self.servo2, [0]),
			Motor("/shoulder_horizontal_left", self.servo3, [1]),
			Motor("/upper_arm_right_rotation", self.servo3, [2]),
			Motor("/elbow_right", self.servo3, [3]),
			Motor("/lower_arm_right_rotation", self.servo3, [4]),
			Motor("/wrist_right", self.servo3, [6]),
			Motor("/shoulder_vertical_right", self.servo3, [7, 9]),
			Motor("/shoulder_horizontal_right", self.servo3, [5, 8])
		]

		# Maps motor-name (as string) to motor-object
		self.motor_map = { motor.name : motor for motor in self.motors }

		self.get_logger().warn("Info: passed __init__")


	def motor_settings_callback(self, request, response):

		try:
			motor = self.motor_map.get(request.motor_name)
			self.get_logger().info(f"Motor: {str(motor)}")
			for port in motor.ports:
				motor.servo.set_pulse_width(port, request.pulse_width_min, request.pulse_width_max)
				motor.servo.set_motion_configuration(port, request.velocity, request.acceleration, request.deceleration)
				motor.servo.set_period(port, request.period)
				motor.servo.set_enable(port, request.turned_on)
				motor.servo.set_degree(port, request.rotation_range_min, request.rotation_range_max)

			response.successful = True
			return response

		except Exception as e:
			self.get_logger().warn(f"Error processing message: {str(e)}")
			response.successful = False
			return response
		

	def joint_trajectory_callback(self, msg):

		try:
			if len(msg.joint_names) == 0:
				raise Exception("Sorry, no numbers below zero")
			else:
				for idx in range(len(msg.joint_names)):
					motor = self.motor_map.get(msg.joint_names[idx])
					self.get_logger().info(f"Motor: {str(motor)}")
					value = msg.points[idx].positions[0]
					self.get_logger().info(f"Value: {value}")
					for port in motor.ports:
						motor.servo.set_position(port, value)	

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