"""PC-49 actual assistant methods with isolated service/device boundaries.

AST-load the complete production class without importing ROS, native audio or
installing process-global fake packages. The same interpreter can then run the
real client/API suites without leaked sys.modules stubs.
"""

from __future__ import annotations

import ast
import collections
import concurrent.futures
import importlib.util
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from ros_stubs import install_ros_stubs

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ros_packages/voice_assistant/voice_assistant/assistant.py"


class ManualTimer:
    def __init__(self, interval, callback):
        self.interval = interval
        self.callback = callback
        self.cancelled = False
        self.daemon = False

    def start(self):
        pass

    def cancel(self):
        self.cancelled = True

    def fire(self, *, race=False):
        if race or not self.cancelled:
            self.callback()


class ManualExecutor:
    def __init__(self):
        self.pending = collections.deque()

    def create_task(self, callback):
        self.pending.append(callback)

    def run(self):
        while self.pending:
            self.pending.popleft()()


@pytest.fixture
def assistant_node(monkeypatch):
    helper_path = ROOT / "pib_hermes_config/pib_hermes_config/turn_taking.py"
    spec = importlib.util.spec_from_file_location(
        "_pc49_actual_turn_taking", helper_path
    )
    assert spec is not None and spec.loader is not None
    helper = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(__import__("sys").modules, spec.name, helper)
    spec.loader.exec_module(helper)
    live_spec = importlib.util.spec_from_file_location(
        "_pc49_actual_live_interaction",
        ROOT / "pib_hermes_config/pib_hermes_config/live_interaction.py",
    )
    assert live_spec is not None and live_spec.loader is not None
    live = importlib.util.module_from_spec(live_spec)
    monkeypatch.setitem(__import__("sys").modules, live_spec.name, live)
    live_spec.loader.exec_module(live)

    class _TurnBusy(Exception):
        def __init__(self, reason: str) -> None:
            self.reason = reason

    class _TextTurnRegistry:
        def start(self, chat_id: str, stop) -> str:
            return "abcd1234"

        def finish(self, turn_id: str) -> None:
            return None

        def get(self, turn_id: str):
            return None

        def cancel(self, turn_id: str):
            return False, ""

    install_ros_stubs()

    ns = {
        "__name__": "_pc49_actual_assistant",
        "Node": object,
        "typed_text_joins_live": live.typed_text_joins_live,
        "TextTurnRegistry": _TextTurnRegistry,
        "TurnBusy": _TurnBusy,
        "STATE_RUNNING": "running",
        "STATE_CANCELLED": "cancelled",
        "turn_state_event": lambda *args, **kwargs: "{}",
        "CancelChatTurn": SimpleNamespace,
        "threading": SimpleNamespace(Timer=ManualTimer, Event=threading.Event),
        "StreamingSpeech": helper.StreamingSpeech,
        "authored_filler": helper.authored_filler,
        "subprocess_fallback_phrase": helper.subprocess_fallback_phrase,
        "first_token_budget_ms": helper.first_token_budget_ms,
        "transcript_opens_turn": helper.transcript_opens_turn,
        "answer_uses_fallback": lambda text: False,
        "FALLBACK_REPLY": "fallback",
        "START_SIGNAL_FILE": "/start.wav",
        "STOP_SIGNAL_FILE": "/stop.wav",
        "MAX_SILENT_SECONDS_BEFORE": 8.0,
        "_CLEAR_PLAYBACK_ACK_LOG_SECONDS": 5.0,
        "ClearPlaybackQueue": SimpleNamespace(Request=SimpleNamespace),
        "VOICE_MODE_LIVE": "live",
        "channel_turn_on_allowed": lambda *args: True,
        "live_session_ends_on_handover": lambda *args: False,
    }
    assistant_stub = types.ModuleType("voice_assistant.assistant")
    for key in (
        "threading",
        "StreamingSpeech",
        "MAX_SILENT_SECONDS_BEFORE",
        "START_SIGNAL_FILE",
        "STOP_SIGNAL_FILE",
        "FALLBACK_REPLY",
        "_CLEAR_PLAYBACK_ACK_LOG_SECONDS",
    ):
        setattr(assistant_stub, key, ns[key])
    assistant_stub.voice_assistant_client = SimpleNamespace()
    monkeypatch.setitem(sys.modules, "voice_assistant.assistant", assistant_stub)

    from voice_assistant.assistant_latency_mixin import AssistantLatencyMixin
    from voice_assistant.assistant_turn_callbacks_mixin import (
        AssistantTurnCallbacksMixin,
    )

    ns["AssistantLatencyMixin"] = AssistantLatencyMixin
    ns["AssistantTurnCallbacksMixin"] = AssistantTurnCallbacksMixin

    tree = ast.parse(SOURCE.read_text())
    cls = next(
        n
        for n in tree.body
        if isinstance(n, ast.ClassDef) and n.name == "VoiceAssistantNode"
    )
    module = ast.Module(
        body=[
            ast.ImportFrom(
                module="__future__", names=[ast.alias(name="annotations")], level=0
            ),
            cls,
        ],
        type_ignores=[],
    )
    ast.fix_missing_locations(module)
    exec(compile(module, str(SOURCE), "exec"), ns)
    klass = ns["VoiceAssistantNode"]
    node = klass.__new__(klass)
    node.executor = ManualExecutor()
    node.get_logger = MagicMock(return_value=MagicMock())
    node.state = SimpleNamespace(turned_on=False, chat_id="chat-a", speaking=False)
    node.gemini_loop = SimpleNamespace(is_listening=False, stop=MagicMock())
    personality = SimpleNamespace(
        gender="Female",
        language="German",
        pause_threshold=0.5,
        personality_id="p1",
        thinking_filler="",
        voice_start_mode="turn_based",
        live_model=None,
        stt_engine="cloud_whisper",
        channel="direct",
    )
    node.personality = personality
    ns["voice_assistant_client"] = SimpleNamespace(
        get_personality_from_chat=lambda chat: (True, personality)
    )
    assistant_stub.voice_assistant_client = ns["voice_assistant_client"]
    node.cycle = 0
    node.turning_off = False
    node.chat_id_to_is_listening = {"chat-a": True, "chat-b": True}
    node.chat_id_to_stop_chat = {}
    node.waiting_for_transcribed_text = False
    node.code_visual_queue = collections.deque()
    node.final_chat_response_received = False
    node.is_executing_program = False
    node._streaming_speech = helper.StreamingSpeech()
    node._filler_timer = None
    node._filler_lock = threading.Lock()
    node._first_token_seen = False
    node._subprocess_fallback_spoken = False
    node._turn_has_playback = False
    node._answer_audio_pending = False
    node._playback_epoch = 0
    node._pending_drain_watchdog = None
    node._answer_drain_max_seconds = 90.0
    node._text_turn_registry = _TextTurnRegistry()
    node._turn_trace_publisher = MagicMock()
    node._stop_active_chat_goal = MagicMock()
    node._publish_text_turn_state = MagicMock()
    for name in (
        "play_audio_from_file",
        "play_audio_from_speech",
        "record_audio",
        "chat",
        "clear_playback_queue",
        "stop_recording",
        "stop_program_execution",
        "run_program",
        "_publish_voice_state",
        "_refresh_personality_turn_taking",
        "_open_latency_turn",
        "_close_latency_turn",
        "_mark_first_sentence_flushed",
    ):
        setattr(node, name, MagicMock())
    node.chat_is_listening_publisher = MagicMock()
    ns["ChatIsListening"] = SimpleNamespace
    try:
        yield node
    finally:
        node._cancel_pending_drain_watchdog()
        node._cancel_filler_timer()


@pytest.mark.parametrize("on", [False, True])
@pytest.mark.parametrize("chat", ["chat-a", "chat-b"])
@pytest.mark.parametrize("ready", [False, True])
def test_typed_chat_routing_matrix(assistant_node, on, chat, ready):
    node = assistant_node
    node.state.turned_on = on
    node.chat_id_to_is_listening[chat] = ready
    response = SimpleNamespace(successful=False)
    node.send_chat_message(SimpleNamespace(chat_id=chat, content="hello"), response)
    assert response.successful is ready
    if not ready:
        node.chat.assert_not_called()
        node.play_audio_from_file.assert_not_called()
        return
    node.chat.assert_called_once()
    spoken = on and chat == "chat-a"
    assert node.chat.call_args.args[2] is spoken
    assert node.play_audio_from_file.called is spoken
    node.play_audio_from_speech.assert_not_called()
    node.record_audio.assert_not_called()


def test_off_final_callback_never_reopens_audio(assistant_node):
    node = assistant_node
    node._answer_audio_pending = True
    node.on_final_sentence_played()
    assert not node._answer_audio_pending
    node.play_audio_from_file.assert_not_called()
    node.record_audio.assert_not_called()


def test_old_callback_cannot_clear_new_turn_in_same_on_cycle(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    old = node.if_cycle_not_changed(
        node.on_final_sentence_played, require_voice_on=True
    )
    node._start_spoken_turn()
    node._answer_audio_pending = True
    old()
    assert node._answer_audio_pending
    node.play_audio_from_file.assert_not_called()


def test_final_callback_is_once_only(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._answer_audio_pending = True
    final = node.if_cycle_not_changed(
        node.on_final_sentence_played, require_voice_on=True
    )
    final()
    final()
    assert not node._answer_audio_pending
    node.play_audio_from_file.assert_called_once()


def test_off_teardown_and_pending_handover_use_actual_update_state(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._answer_audio_pending = True
    old = node.if_cycle_not_changed(
        node.on_final_sentence_played, require_voice_on=True
    )
    assert node.update_state(False)
    assert not node.state.turned_on and not node._answer_audio_pending
    assert node.turning_off
    assert node.update_state(True, "chat-b") is False
    cleared = node.clear_playback_queue.call_args.args[0]
    cleared()
    assert not node.turning_off
    node.play_audio_from_file.assert_not_called()
    assert node.update_state(True, "chat-b")
    node._answer_audio_pending = True
    old()
    assert node._answer_audio_pending
    assert node.state.chat_id == "chat-b"


def test_pending_guard_is_silent_and_watchdog_fails_off(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._answer_audio_pending = True
    node.on_start_signal_played()
    timer = node._pending_drain_watchdog
    assert timer is not None
    node.on_start_signal_played()
    assert node._pending_drain_watchdog is timer  # repeated signals cannot postpone it
    node.play_audio_from_file.assert_not_called()
    node.record_audio.assert_not_called()
    timer.fire()
    assert node.state.turned_on  # timers only post to the serialized ROS executor
    node.executor.run()
    assert not node.state.turned_on
    assert not node._answer_audio_pending
    node.clear_playback_queue.assert_called_once()
    node.play_audio_from_file.assert_not_called()
    node.record_audio.assert_not_called()


def test_stale_watchdog_does_not_cancel_or_untrack_new_drain(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._answer_audio_pending = True
    node._arm_pending_drain_watchdog()
    old = node._pending_drain_watchdog
    node._start_spoken_turn()
    node._answer_audio_pending = True
    node._arm_pending_drain_watchdog()
    new = node._pending_drain_watchdog
    old.fire(race=True)
    node.executor.run()
    assert node._pending_drain_watchdog is new
    assert node.state.turned_on and node._answer_audio_pending
    node.clear_playback_queue.assert_not_called()


def test_empty_final_after_feedback_waits_for_real_barrier(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node.on_sentence_received("Hallo.", False)
    assert node._answer_audio_pending
    node.on_sentence_received("", True)
    assert node.state.turned_on and node.final_chat_response_received
    node.play_audio_from_file.assert_not_called()
    assert node.play_audio_from_speech.call_args.args[0] == ""
    assert node._pending_drain_watchdog is not None
    callback = node.play_audio_from_speech.call_args.args[3]
    callback()
    assert not node._answer_audio_pending
    node.play_audio_from_file.assert_called_once()


def test_final_clause_arms_drain_watchdog(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node.on_sentence_received("Hallo.", True)
    assert node._pending_drain_watchdog is not None
    assert node._answer_audio_pending


def test_program_finish_waits_for_answer_audio(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._turn_has_playback = True
    node._answer_audio_pending = True
    node.final_chat_response_received = True
    node.is_executing_program = True
    node.on_stopped_executing_program()
    node.play_audio_from_file.assert_not_called()
    assert node.play_audio_from_speech.call_args.args[0] == ""


def test_old_filler_cannot_enter_new_turn(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node.personality.thinking_filler = "Please wait."
    node._arm_filler()
    old = node._filler_timer
    node._start_spoken_turn()
    old.fire(race=True)
    node.executor.run()
    node.play_audio_from_speech.assert_not_called()


@pytest.mark.parametrize(
    "method,args",
    [
        ("on_sentence_received", ("late audio", True)),
        ("on_code_visual_received", ("late program", True)),
        ("on_stopped_executing_program", ()),
        ("_finish_spoken_turn", ()),
    ],
)
def test_off_direct_callbacks_have_no_audio_or_program_side_effect(
    assistant_node, method, args
):
    node = assistant_node
    getattr(node, method)(*args)
    node.play_audio_from_file.assert_not_called()
    node.play_audio_from_speech.assert_not_called()
    node.run_program.assert_not_called()
    node.record_audio.assert_not_called()


@pytest.mark.parametrize("outcome", ["success", "error", "cancelled", "no_response"])
def test_clear_only_reports_a_real_service_ack(assistant_node, outcome):
    node = assistant_node
    future = concurrent.futures.Future()
    node.clear_playback_queue_client = SimpleNamespace(
        call_async=lambda request: future
    )
    callback = MagicMock()
    type(node).clear_playback_queue(node, callback)
    if outcome == "success":
        future.set_result(SimpleNamespace())
    elif outcome == "error":
        future.set_exception(RuntimeError("service failure"))
    elif outcome == "cancelled":
        future.cancel()
    else:
        future.set_result(None)
    assert callback.called is (outcome == "success")


def test_subprocess_fallback_notice_speaks_builtin_german(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._speak_subprocess_fallback_notice()
    node.executor.run()
    node.play_audio_from_speech.assert_called_once()
    assert "Einen Moment" in node.play_audio_from_speech.call_args.args[0]


def test_subprocess_fallback_notice_uses_authored_filler(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node.personality.thinking_filler = "Custom cue."
    node._speak_subprocess_fallback_notice()
    node.executor.run()
    assert node.play_audio_from_speech.call_args.args[0] == "Custom cue."


def test_subprocess_fallback_notice_skipped_when_off(assistant_node):
    node = assistant_node
    node.state.turned_on = False
    node._speak_subprocess_fallback_notice()
    node.executor.run()
    node.play_audio_from_speech.assert_not_called()


def test_subprocess_fallback_notice_skipped_when_turning_off(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node.turning_off = True
    node._speak_subprocess_fallback_notice()
    node.executor.run()
    node.play_audio_from_speech.assert_not_called()


def test_subprocess_fallback_notice_only_once(assistant_node):
    node = assistant_node
    node.state.turned_on = True
    node._speak_subprocess_fallback_notice()
    node._speak_subprocess_fallback_notice()
    node.executor.run()
    node.play_audio_from_speech.assert_called_once()


def _call_real_refresh(node) -> None:
    """Run the real refresh method; the fixture stubs it and the harness lacks one helper."""
    method = type(node)._refresh_personality_turn_taking
    method.__globals__.setdefault(
        "pause_threshold_now",
        lambda current, fresh: current if fresh is None else fresh,
    )
    method(node)


def test_an_edited_voice_applies_to_the_next_turn(assistant_node):
    """The dialog's Voice dropdown writes gender; it must reach the running assistant.

    The refresh used to read only the pause threshold and the filler, so a changed voice
    stayed inaudible until a restart and looked like the setting had no effect.
    """
    from voice_assistant import assistant as assistant_stub

    node = assistant_node
    node.personality.gender = "Female"
    fresh = SimpleNamespace(pause_threshold=0.5, thinking_filler="", gender="Male")
    assistant_stub.voice_assistant_client.get_personality = lambda _pid: (True, fresh)

    # The fixture stubs the method on the instance; call the real one.
    _call_real_refresh(node)

    assert node.personality.gender == "Male"


def test_a_missing_stored_voice_keeps_the_current_one(assistant_node):
    from voice_assistant import assistant as assistant_stub

    node = assistant_node
    node.personality.gender = "Male"
    fresh = SimpleNamespace(pause_threshold=0.5, thinking_filler="", gender=None)
    assistant_stub.voice_assistant_client.get_personality = lambda _pid: (True, fresh)

    _call_real_refresh(node)

    assert node.personality.gender == "Male"


def test_wake_word_standby_waits_for_hey_pib_and_runs_wake_up_and_go_sleep_programs(
    assistant_node,
):
    from pib_hermes_config.wake_word import matches_wake_word, normalize_program_choice

    assert matches_wake_word("Hey, pib!") is True
    assert matches_wake_word("hey pib wie geht es dir") is True
    assert matches_wake_word("Hallo pip") is True
    assert matches_wake_word("Guten Morgen") is False
    assert normalize_program_choice("  prog-42  ") == "prog-42"
    assert normalize_program_choice("") is None
    assert normalize_program_choice(None) is None

    node = assistant_node
    node.personality.use_wake_word = True
    node.personality.wake_up_program = "prog-wake"
    node.personality.go_sleep_program = "prog-sleep"
    node.personality.voice_start_mode = "live"
    node.personality.live_model = "gemini-3.8-live"
    node.personality.live_idle_timeout = 60
    node._cloud_session_allowed = lambda: True

    started_live = []
    node.gemini_loop = SimpleNamespace(
        is_listening=False,
        start=lambda **kw: started_live.append(kw),
        stop=MagicMock(),
    )
    dispatched_programs = []
    node.run_program_by_number = lambda p: dispatched_programs.append(p) if p else None

    assert node.update_state(True, "chat-a") is True
    assert node.state.turned_on is True
    assert node._waiting_for_wake_word is True
    assert node._dialog_active is False
    assert started_live == []
    assert dispatched_programs == []
    assert node.record_audio.call_count == 1
    assert node.record_audio.call_args.kwargs.get("stt_engine") == "local_whisper"

    # Unrelated speech re-arms wake-word listening without waking up
    wake_cb = node.record_audio.call_args.args[3]
    wake_cb("Heute ist schönes Wetter")
    assert node._waiting_for_wake_word is True
    assert started_live == []
    assert dispatched_programs == []
    assert node.record_audio.call_count == 2

    # Saying "Hey, pib!" triggers wake_up_program and starts the live dialog
    wake_cb2 = node.record_audio.call_args.args[3]
    wake_cb2("Hey, pib!")
    assert node._waiting_for_wake_word is False
    assert node._dialog_active is True
    assert dispatched_programs == ["prog-wake"]
    assert len(started_live) == 1
    assert started_live[0]["chat_id"] == "chat-a"

    # When the live session ends on idle timeout, go_sleep_program runs and it returns to standby
    node._on_cloud_live_session_end("idle_timeout")
    assert dispatched_programs == ["prog-wake", "prog-sleep"]
    assert node.state.turned_on is True
    assert node._waiting_for_wake_word is True
    assert node._dialog_active is False
