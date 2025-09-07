from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="voice_assistant", executable="assistant"),
            Node(package="voice_assistant", executable="audio_player"),
            Node(package="voice_assistant", executable="audio_recorder"),
            Node(package="voice_assistant", executable="chat"),
            Node(package="voice_assistant", executable="token_service"),
            ExecuteProcess(
                cmd=["/app/ros2_ws/install/voice_assistant/share/voice_assistant/langchain_pib/langchain_proxy.py"],
                name="langchain_proxy",
                output="screen",
            ),
        ]
    )
