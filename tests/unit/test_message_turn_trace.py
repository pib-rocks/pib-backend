"""PR-2072: compact turnTrace on chat message meta."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from test_chat_hermes_routing import chat_module, chat_node  # noqa: E402

from voice_assistant.turn_latency import (  # noqa: E402
    TURN_TRACE_META_MAX_BYTES,
    message_turn_trace,
    turn_trace_meta_bytes,
)


def test_completed_turn_trace_has_integer_ms_and_omits_missing_stages():
    trace = message_turn_trace(
        {
            "ros_service_recv": 0.0,
            "first_token": 332.1,
            "first_sentence_flushed": 349.6,
            "turn_end": 1400.4,
        }
    )
    assert trace == {
        "recv": 0,
        "first_token": 332,
        "first_sentence": 350,
        "done": 1400,
    }
    assert 0 not in (trace.get("first_token"), trace.get("done"))
    assert "stream_done" not in trace


def test_recv_prefers_text_received_over_service_recv():
    trace = message_turn_trace(
        {
            "text_received": 12.4,
            "ros_service_recv": 0.0,
            "first_token": 100.0,
        }
    )
    assert trace["recv"] == 12
    assert trace["first_token"] == 100
    assert "first_sentence" not in trace
    assert "done" not in trace


def test_done_falls_back_to_stream_done_when_turn_end_is_absent():
    trace = message_turn_trace(
        {
            "ros_service_recv": 0.0,
            "stream_done": 999.2,
        }
    )
    assert trace == {"recv": 0, "done": 999}


def test_cancelled_turn_trace_lists_only_reached_stages():
    trace = message_turn_trace(
        {
            "text_received": 5.0,
            "first_token": 120.0,
        },
        cancelled=True,
    )
    assert trace == {
        "recv": 5,
        "first_token": 120,
        "cancelled": True,
    }
    assert "done" not in trace
    assert "first_sentence" not in trace


def test_tool_rounds_are_included_when_tools_ran():
    trace = message_turn_trace(
        {"first_token": 10.0, "turn_end": 50.0},
        tool_rounds=2,
        tool_total_ms=45.7,
    )
    assert trace["toolRounds"] == 2
    assert trace["toolTotalMs"] == 46
    assert trace["first_token"] == 10


def test_turn_trace_payload_stays_within_one_kilobyte():
    heavy_tools = [
        {"name": f"tool_{index}", "ok": True, "durationMs": 12.5, "argNames": ["x"]}
        for index in range(40)
    ]
    stages = {
        "text_received": 1.0,
        "first_token": 2.0,
        "first_sentence_flushed": 3.0,
        "turn_end": 4.0,
    }
    trace = message_turn_trace(
        stages,
        tool_rounds=40,
        tool_total_ms=500.0,
        tool_calls=heavy_tools,
        cancelled=True,
    )
    payload = json.dumps(trace, separators=(",", ":"), sort_keys=True).encode("utf-8")
    assert len(payload) <= TURN_TRACE_META_MAX_BYTES
    meta = {
        "source": "hermes",
        "model": "gemini-3.8-flash",
        "turnTrace": trace,
    }
    assert turn_trace_meta_bytes(meta) > len(payload)


def test_message_turn_trace_builder_is_fast_enough():
    stages = {"text_received": float(index) for index in range(20)} | {
        "first_token": 332.0,
        "first_sentence_flushed": 400.0,
        "turn_end": 1400.0,
    }
    tool_calls = [
        {"name": "set_led", "ok": True, "durationMs": 3.0, "argNames": []}
        for _ in range(30)
    ]
    started = time.perf_counter()
    for _ in range(500):
        message_turn_trace(
            stages,
            tool_rounds=8,
            tool_total_ms=120.0,
            tool_calls=tool_calls,
        )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    assert elapsed_ms < 20.0


def test_chat_node_turn_meta_carries_turn_trace(chat_module, chat_node):
    """Completed turn: turnTrace is additive to existing PC-82 meta."""
    import json
    from unittest.mock import MagicMock

    node = chat_node
    node.get_logger = MagicMock(return_value=MagicMock())
    node._turn_trace_publisher = MagicMock()
    node._reset_turn_meta()

    node._record_turn_trace(json.dumps({"stage": "end_of_speech", "ts_ms": 1000.0}))
    node._record_turn_trace(json.dumps({"stage": "text_received", "ts_ms": 1012.0}))
    node._record_turn_trace(json.dumps({"stage": "request_sent", "ts_ms": 1084.5}))
    node._emit_turn_path(chat_id="chat-meta", path="pib.Cloud", model="pib-cloud")
    node._record_turn_trace(json.dumps({"stage": "first_token", "ts_ms": 1332.1}))
    node._record_turn_trace(
        json.dumps({"stage": "first_sentence_flushed", "ts_ms": 1350.0})
    )
    node._record_turn_trace(json.dumps({"stage": "turn_end", "ts_ms": 2400.0}))

    meta = node._turn_meta()

    assert meta["source"] == "pib-cloud"
    assert meta["stagesMs"]["first_token"] == 332.1
    trace = meta["turnTrace"]
    assert trace["recv"] == 12
    assert trace["first_token"] == 332
    assert trace["first_sentence"] == 350
    assert trace["done"] == 1400
    assert "cancelled" not in trace


def test_chat_node_cancelled_turn_meta_lists_reached_stages_only(
    chat_module, chat_node
):
    import json
    from unittest.mock import MagicMock

    node = chat_node
    node.get_logger = MagicMock(return_value=MagicMock())
    node._reset_turn_meta()
    node._record_turn_trace(json.dumps({"stage": "end_of_speech", "ts_ms": 1000.0}))
    node._record_turn_trace(json.dumps({"stage": "text_received", "ts_ms": 1005.0}))
    node._emit_turn_path(chat_id="chat-cancel", path="Hermes", model="gemini-3.8-flash")
    node._record_turn_trace(json.dumps({"stage": "first_token", "ts_ms": 1120.0}))
    node._turn_trace_cancelled = True

    meta = node._turn_meta()
    trace = meta["turnTrace"]
    assert trace["cancelled"] is True
    assert trace["recv"] == 5
    assert trace["first_token"] == 120
    assert "done" not in trace
    assert "first_sentence" not in trace
