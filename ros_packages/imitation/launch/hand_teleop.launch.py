"""Launch hand teleoperation in non-actuating dry-run mode by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_motion = LaunchConfiguration("enable_motion")
    arm = LaunchConfiguration("arm")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("arm", default_value="right"),
            Node(
                package="imitation",
                executable="hand_teleop",
                name="hand_teleop",
                output="screen",
                parameters=[{"enable_motion": enable_motion, "arm": arm}],
            ),
        ]
    )
