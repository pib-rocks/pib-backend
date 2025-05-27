from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="ros_audio_io", executable="audio_streamer"),
            Node(package="ros_audio_io", executable="doa_publisher"),
        ]
    )
