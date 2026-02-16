from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="motors", executable="motor_control"),
            Node(package="motors", executable="motor_current"),
        ]
    )
