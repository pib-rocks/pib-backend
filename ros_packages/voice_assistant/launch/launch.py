from launch_ros.actions import Node

from launch import LaunchDescription
import os


def generate_launch_description():
    # Surfaced so ChatNode / hermes_agent_client can tune tool-using turn headroom
    # without a redeploy. Default matches hermes_agent_client.DEFAULT_TIMEOUT_SECONDS.
    hermes_timeout = os.environ.get("PIB_HERMES_TIMEOUT", "120")

    return LaunchDescription(
        [
            Node(package="voice_assistant", executable="assistant"),
            Node(package="voice_assistant", executable="audio_player"),
            Node(package="voice_assistant", executable="audio_recorder"),
            Node(
                package="voice_assistant",
                executable="chat",
                additional_env={"PIB_HERMES_TIMEOUT": hermes_timeout},
            ),
            Node(package="voice_assistant", executable="token_service"),
        ]
    )
