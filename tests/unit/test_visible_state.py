"""Visible listening, thinking and speaking, and who holds the voice."""

from pathlib import Path

import pytest

from pib_hermes_config.visible_state import (
    ANSWER_GESTURE_TOOLS,
    LED_BY_PHASE,
    PHASE_DEGRADED,
    PHASE_FALLBACK,
    PHASE_IDLE,
    PHASE_LISTENING,
    PHASE_SPEAKING,
    PHASE_THINKING,
    answer_uses_fallback,
    engine_is_fallback,
    issue_answer_gesture,
    read_key_store_mode,
    resolve_visible_state,
)
from public_api_client.hermes_agent_client import FALLBACK_REPLY

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSISTANT = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "assistant.py"
)
EXPRESSION = (
    REPO_ROOT / "ros_packages" / "display" / "display" / "expression_manager.py"
)
RING = REPO_ROOT / "ros_packages" / "ros_audio_io" / "ros_audio_io" / "doa_publisher.py"
DIRECT = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "direct_tool_loop.py"
)
AUDIO_LOOP = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "audio_loop.py"
)
PLAYER = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "audio_player.py"
)
RECORDER = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "audio_recorder.py"
)
STATE_MSG = REPO_ROOT / "ros_packages" / "datatypes" / "msg" / "VoiceAssistantState.msg"
SKILL = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "skills"
    / "pib-robot-control"
    / "SKILL.md"
)


def _state(**overrides):
    fields = dict(
        turned_on=True,
        chat_id="chat-1",
        personality_id="pers-1",
        personality_name="Ada",
        listening=False,
        listening_chat_id="chat-1",
        speaking=False,
        using_fallback=False,
        operating_mode="unlocked",
    )
    fields.update(overrides)
    return resolve_visible_state(**fields)


def test_three_states_come_from_chat_is_listening_and_voice_state():
    listening = _state(listening=True)
    assert listening.phase == PHASE_LISTENING
    assert listening.expression == PHASE_LISTENING
    assert listening.led_mode == "listen"
    assert listening.display_text == "Ada\nlistening"
    assert listening.holder_personality_id == "pers-1"
    assert listening.holder_name == "Ada"

    thinking = _state(listening=False, speaking=False)
    assert thinking.phase == PHASE_THINKING
    assert thinking.led_mode == "think"
    assert thinking.display_text == "Ada\nthinking"

    speaking = _state(listening=True, speaking=True)
    assert speaking.phase == PHASE_SPEAKING
    assert speaking.led_mode == "speak"
    assert speaking.expression == PHASE_SPEAKING
    assert "speaking" in speaking.display_text


def test_listening_for_another_chat_is_not_the_holders_state():
    state = _state(listening=True, listening_chat_id="chat-2")
    assert state.phase == PHASE_THINKING


def test_voice_off_is_idle_and_names_nobody():
    state = _state(turned_on=False, listening=True, speaking=True)
    assert state.phase == PHASE_IDLE
    assert state.holder_personality_id == ""
    assert state.holder_name == ""
    assert state.led_mode is None
    assert state.expression is None
    assert state.display_text is None


def test_degraded_and_fallback_come_from_the_same_function():
    degraded = _state(turned_on=False, operating_mode=PHASE_DEGRADED)
    assert degraded.phase == PHASE_DEGRADED
    assert degraded.expression == PHASE_DEGRADED
    assert degraded.led_mode == LED_BY_PHASE[PHASE_DEGRADED]
    assert degraded.display_text == "degraded"

    fallback_idle = _state(
        turned_on=False, using_fallback=True, operating_mode="unlocked"
    )
    assert fallback_idle.phase == PHASE_FALLBACK
    assert fallback_idle.led_mode == "trace"

    spoken_fallback = _state(speaking=True, using_fallback=True)
    assert spoken_fallback.phase == PHASE_FALLBACK
    assert spoken_fallback.holder_name == "Ada"
    assert spoken_fallback.display_text == "Ada\nfallback"

    still_speaking = _state(speaking=True, using_fallback=False)
    assert still_speaking.phase == PHASE_SPEAKING

    # A locked store during a live local turn still shows the turn, and names degraded.
    local = _state(listening=True, operating_mode=PHASE_DEGRADED)
    assert local.phase == PHASE_LISTENING
    assert local.display_text == "Ada\nlistening\ndegraded"

    # Degraded wins over fallback when the voice is idle.
    both = _state(turned_on=False, using_fallback=True, operating_mode=PHASE_DEGRADED)
    assert both.phase == PHASE_DEGRADED


def test_fallback_sentence_and_engine_backend():
    assert answer_uses_fallback(FALLBACK_REPLY, FALLBACK_REPLY) is True
    first_clause = FALLBACK_REPLY.split(".")[0] + "."
    assert answer_uses_fallback(first_clause, FALLBACK_REPLY) is True
    assert answer_uses_fallback("Hello there.", FALLBACK_REPLY) is False
    assert engine_is_fallback("fallback") is True
    assert engine_is_fallback("supertone-supertonic-3") is False


def test_cached_key_store_mode_returns_before_the_read_finishes(monkeypatch):
    import time

    import pib_hermes_config.visible_state as visible_state

    def slow_read(**_kwargs):
        time.sleep(0.2)
        return "degraded"

    monkeypatch.setattr(visible_state, "read_key_store_mode", slow_read)
    with visible_state._mode_lock:
        visible_state._mode_cache["value"] = None
        visible_state._mode_cache["at"] = 0.0
        visible_state._mode_refreshing = False

    started = time.monotonic()
    assert visible_state.cached_key_store_mode() is None
    assert time.monotonic() - started < 0.1

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if visible_state.cached_key_store_mode() == "degraded":
            break
        time.sleep(0.02)
    assert visible_state.cached_key_store_mode() == "degraded"


def test_unreadable_key_store_is_not_called_degraded():
    def opener(*_args, **_kwargs):
        raise OSError("down")

    assert read_key_store_mode(opener=opener) is None


def test_a_gesture_during_an_answer_is_an_mcp_tool_call():
    calls = []

    def execute(name, arguments):
        calls.append((name, arguments))
        return {"ok": True, "result": {"poseName": arguments["pose_name"]}}

    result = issue_answer_gesture("apply_pose", {"pose_name": "Wave"}, execute)
    assert result["ok"] is True
    assert calls == [("apply_pose", {"pose_name": "Wave"})]
    with pytest.raises(ValueError, match="MCP"):
        issue_answer_gesture("capture_image", {}, execute)
    assert ANSWER_GESTURE_TOOLS == frozenset({"apply_pose", "move_motor"})


def test_direct_turn_issues_a_gesture_through_the_mcp_execute():
    import sys

    voice_pkg = REPO_ROOT / "ros_packages" / "voice_assistant"
    if str(voice_pkg) not in sys.path:
        sys.path.insert(0, str(voice_pkg))
    from voice_assistant.direct_tool_loop import run_direct_turn

    calls = []

    def complete(messages, tools):
        if not calls:
            return {
                "text": "",
                "tool_calls": [
                    {
                        "id": "gesture-1",
                        "name": "apply_pose",
                        "arguments": {"pose_name": "Wave"},
                    }
                ],
            }
        return {"text": "Here is a wave.", "tool_calls": []}

    def execute(name, arguments):
        calls.append((name, arguments))
        return {"ok": True}

    answer = "".join(
        run_direct_turn(
            system_prompt="You are pib.",
            user_text="Say hello.",
            history=[],
            tool_calling=True,
            allow_image=False,
            declarations=[{"name": "apply_pose", "description": "pose"}],
            complete=complete,
            execute_tool=execute,
        )
    )
    assert calls == [("apply_pose", {"pose_name": "Wave"})]
    assert answer == "Here is a wave."


def test_surfaces_read_the_same_state():
    assistant = ASSISTANT.read_text(encoding="utf-8")
    face = EXPRESSION.read_text(encoding="utf-8")
    ring = RING.read_text(encoding="utf-8")
    direct = DIRECT.read_text(encoding="utf-8")
    loop = AUDIO_LOOP.read_text(encoding="utf-8")
    player = PLAYER.read_text(encoding="utf-8")
    recorder = RECORDER.read_text(encoding="utf-8")
    message = STATE_MSG.read_text(encoding="utf-8")
    skill = SKILL.read_text(encoding="utf-8")

    for field in ("speaking", "using_fallback", "personality_name"):
        assert field in message
    assert "personality_name" in assistant
    assert "_note_spoken_answer(" in assistant
    assert "set_speaking_listener(" in assistant
    assert "VOICE_USING_FALLBACK_TOPIC" in assistant
    assert "VOICE_USING_FALLBACK_TOPIC" in player
    assert "VOICE_USING_FALLBACK_TOPIC" in recorder
    assert "engine_is_fallback(" in player
    assert "engine_is_fallback(" in recorder

    assert "voice_assistant_state" in face
    assert "chat_is_listening" in face
    assert "resolve_visible_state(" in face
    assert "cached_key_store_mode(" in face
    assert "render_phase_png" in face
    assert '"/voice_activity"' in face

    assert "voice_assistant_state" in ring
    assert "chat_is_listening" in ring
    assert "resolve_visible_state(" in ring
    assert "cached_key_store_mode(" in ring

    assert "issue_answer_gesture(" in direct
    assert "issue_answer_gesture(" in loop
    assert "apply_pose" in skill
    assert "move_motor" in skill
    assert "MCP" in skill
