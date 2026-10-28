"""PR-2120: voice node module decomposition contracts."""

from __future__ import annotations

import ast
import inspect
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "ros_packages" / "voice_assistant" / "voice_assistant"
VOICE_PKG = ROOT / "ros_packages" / "voice_assistant"

if str(VOICE_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_PKG))

from ros_stubs import install_ros_stubs

install_ros_stubs()

CHAT_PY = PKG / "chat.py"
ASSISTANT_PY = PKG / "assistant.py"

CHAT_TURN_TRACE_METHODS = {
    "_install_turn_trace_emitter",
    "_turn_trace_chat_id",
    "_record_tool_started",
    "_record_tool_finished",
    "_record_tool_call",
    "_record_tool_image",
    "_reset_turn_meta",
    "_ensure_turn_trace_anchor",
    "_record_local_turn_stage",
    "_freeze_turn_done_stage",
    "_emit_turn_state",
    "_discard_assistant_reply",
    "_finish_cancelled_turn",
    "_record_turn_trace",
    "_emit_hermes_subprocess_fallback_notice",
    "_turn_meta",
    "_provider_base_url",
    "_finalize_message_meta",
    "_attach_turn_images",
    "_emit_turn_path",
}

CHAT_HERMES_METHODS = {
    "_ensure_hermes_daemon",
    "_preflight_hermes_binary",
    "_persist_streaming_assistant_text",
    "_stream_chunks_to_goal",
    "_remember_first_token_latency",
    "_publish_ready_clauses",
    "_hermes_timeout",
    "_run_hermes_turn",
    "_stream_hermes_turn",
    "_robot_context",
    "_iter_local_tokens",
    "_stream_pib_cloud_turn",
    "_execute_chat_turn",
}

ASSISTANT_LATENCY_METHODS = {
    "_start_spoken_turn",
    "_arm_filler",
    "_speak_subprocess_fallback_notice",
    "_note_first_token",
    "_cancel_filler_timer",
    "_finish_spoken_turn",
    "_trace_source",
    "_open_latency_turn",
    "_mark_text_received",
    "_mark_first_sentence_flushed",
    "_close_latency_turn",
    "_on_turn_trace",
    "_turn_trace_watchdog",
    "_schedule_voice_callback",
    "_invalidate_voice_async_work",
    "_cancel_pending_drain_watchdog",
    "_arm_pending_drain_watchdog",
}

ASSISTANT_CALLBACK_METHODS = {
    "on_start_signal_played",
    "_on_stop_signal_played",
    "on_stopped_recording",
    "on_transcribed_text_received",
    "on_sentence_received",
    "on_code_visual_received",
    "on_stopped_executing_program",
    "on_final_sentence_played",
    "if_cycle_not_changed",
    "digest_goal_handle_future",
    "set_is_listening",
    "_note_spoken_answer",
    "_on_live_speaking",
    "_on_engine_fallback",
    "_on_stt_failure",
    "_republish_visible_state",
    "get_is_listening",
    "stop_chat",
    "barge_in",
    "_sync_voice_state",
    "_publish_voice_state",
    "update_state",
}


def _line_count(path: Path) -> int:
    return len(path.read_text(encoding="utf-8").splitlines())


def _no_logger_exc_info(path: Path) -> None:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if not isinstance(node.func.value, ast.Call):
            continue
        if not (
            isinstance(node.func.value.func, ast.Attribute)
            and node.func.value.func.attr == "get_logger"
        ):
            continue
        for keyword in node.keywords:
            assert keyword.arg not in {
                "exc_info",
                "stack_info",
            }, f"{path.name}:{node.lineno} uses {keyword.arg}= on get_logger()"


def test_chat_and_assistant_modules_under_800_lines():
    assert _line_count(CHAT_PY) < 800, f"chat.py has {_line_count(CHAT_PY)} lines"
    assert (
        _line_count(ASSISTANT_PY) < 800
    ), f"assistant.py has {_line_count(ASSISTANT_PY)} lines"


def test_chat_node_mro_and_method_contract():
    from voice_assistant.chat import ChatNode
    from voice_assistant.chat_hermes_mixin import ChatHermesMixin
    from voice_assistant.chat_turn_trace_mixin import ChatTurnTraceMixin

    mro = ChatNode.__mro__
    assert ChatTurnTraceMixin in mro
    assert ChatHermesMixin in mro

    for name in CHAT_TURN_TRACE_METHODS:
        assert hasattr(ChatNode, name), name
        assert name in ChatTurnTraceMixin.__dict__, name
    for name in CHAT_HERMES_METHODS:
        assert hasattr(ChatNode, name), name
        assert name in ChatHermesMixin.__dict__, name


def test_voice_assistant_node_mro_and_method_contract():
    assistant_tree = ast.parse(ASSISTANT_PY.read_text(encoding="utf-8"))
    cls = next(
        node
        for node in assistant_tree.body
        if isinstance(node, ast.ClassDef) and node.name == "VoiceAssistantNode"
    )
    base_names = [base.id for base in cls.bases if isinstance(base, ast.Name)]
    assert base_names[:2] == [
        "AssistantTurnCallbacksMixin",
        "AssistantLatencyMixin",
    ]

    def mixin_methods(filename: str, class_name: str) -> set[str]:
        tree = ast.parse((PKG / filename).read_text(encoding="utf-8"))
        mixin = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == class_name
        )
        return {
            node.name
            for node in mixin.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    latency_methods = mixin_methods(
        "assistant_latency_mixin.py", "AssistantLatencyMixin"
    )
    callback_methods = mixin_methods(
        "assistant_turn_callbacks_mixin.py", "AssistantTurnCallbacksMixin"
    )

    for name in ASSISTANT_LATENCY_METHODS:
        assert name in latency_methods, name
    for name in ASSISTANT_CALLBACK_METHODS:
        assert name in callback_methods, name


def test_extracted_modules_obey_logger_contract():
    for path in (
        PKG / "chat_turn_trace_mixin.py",
        PKG / "chat_hermes_mixin.py",
        PKG / "assistant_latency_mixin.py",
        PKG / "assistant_turn_callbacks_mixin.py",
    ):
        _no_logger_exc_info(path)


def test_prepare_tool_image_reexported_from_chat():
    from voice_assistant import chat as chat_mod

    assert callable(chat_mod.prepare_tool_image)
    assert inspect.isfunction(chat_mod._model_provider_name)
