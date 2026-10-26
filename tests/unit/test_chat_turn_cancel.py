"""PR-2069: explicit turn id, cancel path, and no partial assistant persistence."""

from __future__ import annotations

import json
import threading
import time
from unittest.mock import MagicMock, patch

import sys
from pathlib import Path

import pytest

_UNIT = Path(__file__).resolve().parent
if str(_UNIT) not in sys.path:
    sys.path.insert(0, str(_UNIT))

from test_chat_hermes_routing import chat_module, chat_node, drive_like_rclpy

from voice_assistant.chat_turn import (
    REASON_ALREADY_FINISHED,
    REASON_TURN_IN_PROGRESS,
    REASON_UNKNOWN_TURN,
    STATE_CANCELLED,
    STATE_COMPLETED,
    STATE_RUNNING,
    new_turn_id,
)


def _turn_state_events(publisher) -> list[dict]:
    events = []
    for call in publisher.publish.call_args_list:
        raw = call[0][0].data
        event = json.loads(raw)
        if event.get("stage") == "turn_state":
            events.append(event)
    return events


def _personality():
    personality = MagicMock()
    personality.message_history = 5
    personality.description = "Du bist pib."
    personality.personality_id = "pers-1"
    personality.channel = "smart"
    personality.voice_mode = "turn_based"
    personality.assistant_model.api_name = "hermes-agent"
    personality.assistant_model.has_image_support = False
    return personality


def test_cancel_before_first_token_removes_no_assistant_row(chat_module, chat_node):
    """Cancel while the model is still cold: user line stays, no assistant create."""
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat
    turn_id = new_turn_id()
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-1"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False
    goal_handle.request.turn_id = turn_id

    blocked = threading.Event()

    def _stream(*_args, **_kwargs):
        blocked.wait(timeout=2)
        yield "never"

    chat_node._turn_trace_publisher = MagicMock()
    with (
        patch.object(
            chat_module.voice_assistant_client,
            "get_personality_from_chat",
            return_value=(True, _personality()),
        ),
        patch.object(hermes_agent_client, "is_warm_daemon_active", return_value=False),
        patch.object(hermes_agent_client, "ensure_profile", return_value="/tmp/p"),
        patch.object(chat_node, "_stream_hermes_turn", side_effect=_stream),
        patch.object(chat_node, "_emit_turn_path"),
        patch.object(chat_node, "_finalize_message_meta"),
        patch.object(chat_node, "_discard_assistant_reply") as discard,
    ):
        t = threading.Thread(
            target=lambda: drive_like_rclpy(chat_node.chat(goal_handle)), daemon=True
        )
        t.start()
        time.sleep(0.05)
        goal_handle.is_cancel_requested = True
        blocked.set()
        t.join(timeout=5)

    goal_handle.canceled.assert_called()
    discard.assert_called_once_with("chat-1")
    states = _turn_state_events(chat_node._turn_trace_publisher)
    assert any(
        e["detail"]["state"] == STATE_CANCELLED and e["detail"]["turn_id"] == turn_id
        for e in states
    )


def test_cancel_during_streaming_deletes_partial_assistant(chat_module, chat_node):
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat
    turn_id = new_turn_id()
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-2"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False
    goal_handle.request.turn_id = turn_id

    def _tokens():
        yield "Hello "
        goal_handle.is_cancel_requested = True
        yield "world. "

    chat_node._turn_trace_publisher = MagicMock()
    chat_node._turn_assistant_message_id = "asst-1"
    with (
        patch.object(
            chat_module.voice_assistant_client,
            "get_personality_from_chat",
            return_value=(True, _personality()),
        ),
        patch.object(hermes_agent_client, "is_warm_daemon_active", return_value=False),
        patch.object(hermes_agent_client, "ensure_profile", return_value="/tmp/p"),
        patch.object(chat_node, "_stream_hermes_turn", return_value=_tokens()),
        patch.object(chat_node, "_emit_turn_path"),
        patch.object(chat_node, "_finalize_message_meta"),
        patch.object(chat_node, "_discard_assistant_reply") as discard,
    ):
        drive_like_rclpy(chat_node.chat(goal_handle))

    discard.assert_called()
    goal_handle.canceled.assert_called()


def test_registry_rejects_second_turn_while_first_runs():
    from voice_assistant.chat_turn import TextTurnRegistry, TurnBusy

    registry = TextTurnRegistry()
    first = registry.start("chat-a", stop=MagicMock())
    with pytest.raises(TurnBusy) as exc:
        registry.start("chat-a", stop=MagicMock())
    assert exc.value.reason == REASON_TURN_IN_PROGRESS
    registry.finish(first)


def test_registry_cancel_unknown_and_finished():
    from voice_assistant.chat_turn import TextTurnRegistry

    registry = TextTurnRegistry()
    ok, reason = registry.cancel("nope")
    assert not ok and reason == REASON_UNKNOWN_TURN

    stop = MagicMock()
    turn_id = registry.start("chat-a", stop=stop)
    registry.finish(turn_id)
    ok, reason = registry.cancel(turn_id)
    assert not ok and reason == REASON_ALREADY_FINISHED


def test_registry_cancel_invokes_stop():
    from voice_assistant.chat_turn import TextTurnRegistry

    registry = TextTurnRegistry()
    stop = MagicMock()
    turn_id = registry.start("chat-a", stop=stop)
    ok, _ = registry.cancel(turn_id)
    assert ok
    stop.assert_called_once()


def test_running_turn_emits_turn_state_on_chat_node(chat_module, chat_node):
    from public_api_client import hermes_agent_client

    Chat = chat_module.Chat
    turn_id = new_turn_id()
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    goal_handle.request = Chat.Goal()
    goal_handle.request.chat_id = "chat-3"
    goal_handle.request.text = "Hi"
    goal_handle.request.generate_code = False
    goal_handle.request.turn_id = turn_id

    chat_node._turn_trace_publisher = MagicMock()
    with (
        patch.object(
            chat_module.voice_assistant_client,
            "get_personality_from_chat",
            return_value=(True, _personality()),
        ),
        patch.object(hermes_agent_client, "is_warm_daemon_active", return_value=False),
        patch.object(hermes_agent_client, "ensure_profile", return_value="/tmp/p"),
        patch.object(chat_node, "_stream_hermes_turn", return_value=iter(["Done. "])),
        patch.object(chat_node, "_emit_turn_path"),
        patch.object(chat_node, "_finalize_message_meta"),
        patch.object(chat_node, "_discard_assistant_reply"),
    ):
        drive_like_rclpy(chat_node.chat(goal_handle))

    states = _turn_state_events(chat_node._turn_trace_publisher)
    assert any(
        e["detail"]["state"] == STATE_RUNNING and e["detail"]["turn_id"] == turn_id
        for e in states
    )
    assert any(
        e["detail"]["state"] == STATE_COMPLETED and e["detail"]["turn_id"] == turn_id
        for e in states
    )
