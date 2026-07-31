from launch_ros.actions import Node

from launch import LaunchDescription
import os


def generate_launch_description():
    # Surfaced so ChatNode / hermes_agent_client can tune tool-using turn headroom
    # without a redeploy. Default matches hermes_agent_client.DEFAULT_TIMEOUT_SECONDS.
    hermes_timeout = os.environ.get("PIB_HERMES_TIMEOUT", "120")
    # One explicit location each, matching the bind mounts in docker-compose.yaml:
    # the CLI wrapper and the profiles dir shared with the flask API. Nothing is
    # probed or auto-detected — if these are wrong, ChatNode's preflight says so.
    hermes_bin = os.environ.get("PIB_HERMES_BIN", "/home/pib/.local/bin/hermes")
    hermes_profiles_dir = os.environ.get(
        "PIB_HERMES_PROFILES_DIR", "/home/pib/.hermes/profiles"
    )

    return LaunchDescription(
        [
            Node(package="voice_assistant", executable="assistant"),
            Node(package="voice_assistant", executable="audio_player"),
            Node(package="voice_assistant", executable="audio_recorder"),
            Node(
                package="voice_assistant",
                executable="chat",
                additional_env={
                    "PIB_HERMES_TIMEOUT": hermes_timeout,
                    "PIB_HERMES_BIN": hermes_bin,
                    "PIB_HERMES_PROFILES_DIR": hermes_profiles_dir,
                },
            ),
            Node(package="voice_assistant", executable="token_service"),
        ]
    )
