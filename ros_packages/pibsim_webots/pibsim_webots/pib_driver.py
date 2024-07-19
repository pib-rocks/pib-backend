import math
from typing import Iterable, Tuple
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PibDriver:
    """
    This driver is the interface between pib's digital twin for Webots and the ROS jointTrajectory topic.
    Every JointTrajectory message published to the topic gets converted into Webots movement instructions.
    """

    def init(self, webots_node, properties):
        """The init method is the Webot/ ROS node counterpart to the python __init__ constructor"""
        self._robot = webots_node.robot
        self._devices = self._initialize_devices()

        rclpy.init(args=None)
        self._node = rclpy.create_node("pib_driver")
        self._node.create_subscription(
            JointTrajectory, "/joint_trajectory", self._trajectory_callback, 10
        )
        self._node.get_logger().info("PibDriver ready")

    MOTOR_TO_DEVICES_MAP = {
        "turn_head_motor": ["head_horizontal"],
        "tilt_forward_motor": ["head_vertical"],
        "upper_arm_left_rotation": ["upper_arm_left"],
        "elbow_left": ["elbow_left"],
        "lower_arm_left_rotation": ["forearm_left"],
        "shoulder_vertical_left": ["shoulder_vertical_left"],
        "shoulder_horizontal_left": ["shoulder_horizontal_left"],
        "upper_arm_right_rotation": ["upper_arm_right"],
        "elbow_right": ["elbow_right"],
        "lower_arm_right_rotation": ["forearm_right"],
        "shoulder_vertical_right": ["shoulder_vertical_right"],
        "shoulder_horizontal_right": ["shoulder_horizontal_right"],
        "thumb_right_opposition": ["thumb_right_opposition"],
        "thumb_right_stretch": ["thumb_right_distal", "thumb_right_proximal"],
        "index_right_stretch": ["index_right_distal", "index_right_proximal"],
        "middle_right_stretch": ["middle_right_distal", "middle_right_proximal"],
        "ring_right_stretch": ["ring_right_distal", "ring_right_proximal"],
        "pinky_right_stretch": ["pinky_right_distal", "pinky_right_proximal"],
        "thumb_left_opposition": ["thumb_left_opposition"],
        "thumb_left_stretch": ["thumb_left_distal", "thumb_left_proximal"],
        "index_left_stretch": ["index_left_distal", "index_left_proximal"],
        "middle_left_stretch": ["middle_left_distal", "middle_left_proximal"],
        "ring_left_stretch": ["ring_left_distal", "ring_left_proximal"],
        "pinky_left_stretch": ["pinky_left_distal", "pinky_left_proximal"],
        "wrist_left": ["wrist_left"],
        "wrist_right": ["wrist_right"],
    }

    def _initialize_devices(self) -> dict:
        """Initialize all webot robot devices associated with the pib robot names"""
        devices = {}

        for pib_motor_name, proto_device_names in self.MOTOR_TO_DEVICES_MAP.items():
            devices[pib_motor_name] = self.find_devices_in_robot(proto_device_names)
        return devices

    def _trajectory_callback(self, joint_trajectory: JointTrajectory):
        """Sets the positions of all devices in a received joint trajectory message."""
        motor_name_position_pairs = self.extract_motor_name_position_pairs(
            joint_trajectory
        )

        for (
            motor_name,
            device_target_position,
        ) in motor_name_position_pairs:
            self.set_device_positions(
                motor_name, self.convert_position(device_target_position)
            )

    def find_devices_in_robot(self, device_names: list) -> list:
        """Retrieves devices from the robot based on a list of device names"""
        device_list = []

        for device_name in device_names:
            device = self._robot.getDevice(device_name)
            if device is not None:
                device_list.append(device)
            else:
                self._node.get_logger().error(
                    f"Failed to retrieve device: {device_name}"
                )
        return device_list

    def convert_position(self, position: float) -> float:
        """Converts degree to radian and adjust for cerebras offset (multiplying degree by 100)."""
        return math.radians(position / 100.0)

    def extract_motor_name_position_pairs(
        self, jt: JointTrajectory
    ) -> Iterable[Tuple[str, int]]:
        """unpacks a jt-message into an iterable of motor_name-position-pairs"""
        motor_names = jt.joint_names
        points: list[JointTrajectoryPoint] = jt.points
        positions = (point.positions[0] for point in points)

        if len(motor_names) != len(points):
            self._node.get_logger().error(
                f"Syntax error within JointTrajectory message: The number of motor names ({len(motor_names)}) doesn't match the number of jointTrajectoryPoints ({len(points)})."
            )
            return []

        motor_name_position_pairs = zip(motor_names, positions)

        return motor_name_position_pairs

    def set_device_positions(self, motor_name: str, device_target_position: float):
        """Sets the position of a device."""
        try:
            for device in self._devices[motor_name]:
                device.setPosition(device_target_position)
            self._node.get_logger().info(
                f"Device '{motor_name}' moved to position {device_target_position}"
            )
        except KeyError:
            self._node.get_logger().error(
                f"Motor name '{motor_name}' not found in devices."
            )

    def step(self):
        """Spins the ROS node once to handle any pending messages or callbacks."""
        rclpy.spin_once(self._node, timeout_sec=0)
