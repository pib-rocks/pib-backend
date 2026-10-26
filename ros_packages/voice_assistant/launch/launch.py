from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import os

from node_supervision import (
    HEALTH_GAVE_UP,
    RespawnPolicy,
    max_respawn_attempts_from_env,
)


def _chat_node(hermes_timeout: str, hermes_bin: str, hermes_profiles_dir: str) -> Node:
    return Node(
        package="voice_assistant",
        executable="chat",
        additional_env={
            "PIB_HERMES_TIMEOUT": hermes_timeout,
            "PIB_HERMES_BIN": hermes_bin,
            "PIB_HERMES_PROFILES_DIR": hermes_profiles_dir,
        },
    )


def _assistant_node() -> Node:
    return Node(package="voice_assistant", executable="assistant")


def _supervised_exit_handler(policy: RespawnPolicy, node_factory):
    """Respawn chat/assistant on crash with bounded backoff and one log line."""

    def on_exit(event, context):
        decision = policy.on_process_exit(event.returncode, context.is_shutdown)
        if decision.log_line:
            print(decision.log_line, flush=True)
        if not decision.respawn:
            if decision.health == HEALTH_GAVE_UP:
                print(
                    f"voice launch supervision: health={decision.health} "
                    f"node={policy.node_name}",
                    flush=True,
                )
            return []
        return [
            LogInfo(msg=decision.log_line),
            TimerAction(
                period=decision.delay_seconds,
                actions=[node_factory()],
            ),
        ]

    return on_exit


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

    max_attempts = max_respawn_attempts_from_env()
    chat_policy = RespawnPolicy("chat", max_attempts=max_attempts)
    assistant_policy = RespawnPolicy("assistant", max_attempts=max_attempts)

    chat = _chat_node(hermes_timeout, hermes_bin, hermes_profiles_dir)
    assistant = _assistant_node()

    return LaunchDescription(
        [
            assistant,
            Node(package="voice_assistant", executable="audio_player"),
            Node(package="voice_assistant", executable="audio_recorder"),
            chat,
            Node(package="voice_assistant", executable="token_service"),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=chat,
                    on_exit=_supervised_exit_handler(
                        chat_policy,
                        lambda: _chat_node(
                            hermes_timeout, hermes_bin, hermes_profiles_dir
                        ),
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=assistant,
                    on_exit=_supervised_exit_handler(
                        assistant_policy,
                        _assistant_node,
                    ),
                )
            ),
        ]
    )
