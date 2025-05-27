from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="voice_assistant", executable="assistant"),
            Node(package="voice_assistant", executable="audio_player"),
            Node(package="voice_assistant", executable="audio_recorder"),
            Node(package="voice_assistant", executable="chat"),
            Node(package="voice_assistant", executable="token_service"),
        ]
    )
