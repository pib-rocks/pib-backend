"""Unit tests for bounded voice node respawn supervision (PR-2070)."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

LAUNCH_DIR = (
    Path(__file__).resolve().parents[2] / "ros_packages" / "voice_assistant" / "launch"
)
SUPERVISION_FILE = LAUNCH_DIR / "node_supervision.py"


def _load_supervision():
    spec = importlib.util.spec_from_file_location(
        "voice_launch_node_supervision", SUPERVISION_FILE
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def supervision():
    return _load_supervision()


def test_respawn_policy_logs_once_per_exit_and_backoffs(supervision, caplog):
    policy = supervision.RespawnPolicy("chat", max_attempts=3)
    caplog.set_level("INFO")

    first = policy.on_process_exit(-6, is_shutdown=False)
    assert first.respawn is True
    assert first.delay_seconds == 1.0
    assert "respawning chat" in first.log_line
    assert first.health == supervision.HEALTH_RESPAWNING

    second = policy.on_process_exit(1, is_shutdown=False)
    assert second.respawn is True
    assert second.delay_seconds == 2.0
    assert "attempt=2/3" in second.log_line

    third = policy.on_process_exit(1, is_shutdown=False)
    assert third.respawn is True
    assert third.delay_seconds == 4.0

    gave_up = policy.on_process_exit(1, is_shutdown=False)
    assert gave_up.respawn is False
    assert gave_up.health == supervision.HEALTH_GAVE_UP
    assert supervision.GAVE_UP_LOG_PREFIX in gave_up.log_line
    assert "after 3 respawns" in gave_up.log_line


def test_respawn_delay_never_exceeds_five_seconds(supervision):
    policy = supervision.RespawnPolicy("assistant", max_attempts=10)
    for _ in range(6):
        decision = policy.on_process_exit(-6, is_shutdown=False)
        if not decision.respawn:
            break
        assert decision.delay_seconds <= supervision.MAX_RESPAWN_DELAY_SECONDS


def test_clean_exit_and_shutdown_do_not_respawn(supervision):
    policy = supervision.RespawnPolicy("chat")
    assert policy.on_process_exit(0, is_shutdown=False).respawn is False
    assert policy.on_process_exit(-6, is_shutdown=True).respawn is False
    assert policy.state.attempts == 0


def test_launch_registers_supervision_for_chat_and_assistant(
    monkeypatch: pytest.MonkeyPatch,
):
    """Launch wires exit handlers only for chat and assistant."""
    launch = types.ModuleType("launch")
    launch.LaunchDescription = lambda entities: types.SimpleNamespace(
        entities=list(entities)
    )
    actions = types.ModuleType("launch.actions")

    class RegisterEventHandler:
        def __init__(self, event_handler):
            self.event_handler = event_handler

    class TimerAction:
        def __init__(self, period, actions):
            self.period = period
            self.actions = actions

    class LogInfo:
        def __init__(self, msg):
            self.msg = msg

    actions.RegisterEventHandler = RegisterEventHandler
    actions.TimerAction = TimerAction
    actions.LogInfo = LogInfo
    event_handlers = types.ModuleType("launch.event_handlers")

    class OnProcessExit:
        def __init__(self, *, target_action, on_exit):
            self.target_action = target_action
            self.on_exit = on_exit

    event_handlers.OnProcessExit = OnProcessExit
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")

    class LaunchNode:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    launch_ros_actions.Node = LaunchNode
    for name, module in {
        "launch": launch,
        "launch.actions": actions,
        "launch.event_handlers": event_handlers,
        "launch_ros": launch_ros,
        "launch_ros.actions": launch_ros_actions,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    monkeypatch.syspath_prepend(str(LAUNCH_DIR))
    launch_file = LAUNCH_DIR / "launch.py"
    spec = importlib.util.spec_from_file_location("voice_launch_file", launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    nodes = [e for e in description.entities if isinstance(e, LaunchNode)]
    executables = {n.kwargs.get("executable") for n in nodes}
    assert executables == {
        "assistant",
        "audio_player",
        "audio_recorder",
        "chat",
        "token_service",
    }

    handlers = [
        e.event_handler
        for e in description.entities
        if isinstance(e, RegisterEventHandler)
    ]
    assert len(handlers) == 2
    supervised = {h.target_action.kwargs["executable"] for h in handlers}
    assert supervised == {"chat", "assistant"}
