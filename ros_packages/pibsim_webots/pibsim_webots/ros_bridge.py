import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS_RIGHT: List[str] = [
    "shoulder_vertical_right",
    "shoulder_horizontal_right",
    "elbow_right",
    "wrist_right",
    "thumb_right_opposition",
    "thumb_right_stretch",
    "index_right_stretch",
    "middle_right_stretch",
    "ring_right_stretch",
    "pinky_right_stretch",
]

_node: Optional[Node] = None
_pub = None

def start(node_name: str = "mcp_ros_bridge", topic: str = "/joint_trajectory"):
    global _node, _pub
    if not rclpy.ok():
        rclpy.init()
    if _node is None:
        _node = Node(node_name)
        _pub = _node.create_publisher(JointTrajectory, topic, 10)
        time.sleep(0.1)  # kurzer Moment für Publisher-Setup

def stop():
    global _node, _pub
    if _node is not None:
        _node.destroy_node()
        _node = None
        _pub = None
    if rclpy.ok():
        rclpy.shutdown()

def publish_right_pose(targets: Dict[str, float]):
    """Erwartet dict {joint_name: position} und publiziert eine JointTrajectory."""
    assert _node is not None and _pub is not None, "Bridge nicht gestartet. Ruf start() auf."
    msg = JointTrajectory()
    msg.joint_names = JOINTS_RIGHT
    msg.header.stamp = _node.get_clock().now().to_msg()

    ns = 0
    points = []
    for j in JOINTS_RIGHT:
        p = JointTrajectoryPoint()
        p.positions = [float(targets.get(j, 0.0))]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = ns
        ns += 10
        points.append(p)
    msg.points = points
    _pub.publish(msg)