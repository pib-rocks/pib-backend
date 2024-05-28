from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="programs", executable="program"),
            Node(package="programs", executable="proxy_program"),
        ]
    )
