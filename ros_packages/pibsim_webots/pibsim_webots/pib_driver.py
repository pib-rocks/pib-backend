import rclpy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PibDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__devices = self.__initialize_devices()
        self.__target_trajectory = JointTrajectory()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("pib_driver")
        self.__node.get_logger().info("Node created")
        self.__node.create_subscription(
            JointTrajectory, "/joint_trajectory", self.__trajectory_callback, 1
        )
        self.__node.get_logger().info("Subscribed to /joint_trajectory")
    
    def __initialize_devices(self) -> dict:
        device_groups = {
            "turn_head_motor": ["head_horizontal"],
            "tilt_forward_motor": ["head_vertical"],
            "shoulder_horizontal_right": ["shoulder_horizontal_right"],
            "shoulder_horizontal_left": ["shoulder_horizontal_left"],
            "shoulder_vertical_right": ["shoulder_vertical_right"],
            "shoulder_vertical_left": ["shoulder_vertical_left"],
            "elbow_right": ["elbow_right"],
            "elbow_left": ["elbow_left"],
            "lower_arm_right_rotation": ["forearm_right"],
            "lower_arm_left_rotation": ["forearm_left"],
            "thumb_right_stretch": ["thumb_right_distal", "thumb_right_proximal"],
            "thumb_left_stretch": ["thumb_left_distal", "thumb_left_proximal"],
            "index_right_stretch": ["index_right_distal", "index_right_proximal"],
            "middle_right_stretch": ["middle_right_distal", "middle_right_proximal"],
            "ring_right_stretch": ["ring_right_distal", "ring_right_proximal"],
            "pinky_right_stretch": ["pinky_right_distal", "pinky_right_proximal"],
            "index_left_stretch": ["index_left_distal", "index_left_proximal"],
            "middle_left_stretch": ["middle_left_distal", "middle_left_proximal"],
            "ring_left_stretch": ["ring_left_distal", "ring_left_proximal"],
            "pinky_left_stretch": ["pinky_left_distal", "pinky_left_proximal"],
        }

        devices = {}
        for name, parts in device_groups.items():
            devices[name] = [self.__robot.getDevice(part) for part in parts]

        return devices
    def __trajectory_callback(self, trajectory: JointTrajectory):
        self.__target_trajectory = trajectory
        self.__node.get_logger().info(
            f"Trajectory received: moving {trajectory.joint_names[0]} to {trajectory.points[0].positions[0]}"
        )

    def __convert_position(self, position: float) -> float:
        return math.radians(position / 100.0)

    def __create_device(self, name: str):
        rot_index = name.find("rota")
        device_name = name[:rot_index - 1] if rot_index != -1 else name

        try:
            self.__devices[name] = [self.__robot.getDevice(device_name)]
            self.__node.get_logger().info(f"Device created: {device_name} (name: {name})")
        except Exception as e:
            self.__node.get_logger().error(f"Failed to create device {name}: {str(e)}")

    def __set_target_positions(self):
        if not self.__target_trajectory.joint_names:
            return

        for index, name in enumerate(self.__target_trajectory.joint_names):
            if name not in self.__devices:
                self.__create_device(name)

            position = self.__convert_position(self.__target_trajectory.points[0].positions[index])

            for device in self.__devices.get(name, []):
                if device:
                    try:
                        device.setPosition(position)
                    except Exception as e:
                        self.__node.get_logger().error(f"Failed to set position for {name}: {str(e)}")


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__set_target_positions()
