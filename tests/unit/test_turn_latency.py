"""Pure-Python tests for per-turn latency collection. No rclpy."""

from __future__ import annotations

import json
import logging
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.turn_latency import (  # noqa: E402
    EIGHT_STAGES,
    TurnTrace,
    append_record,
    format_event,
    parse_event,
    render_line,
)


def test_format_event_round_trip_and_malformed_input_returns_none():
    raw = format_event(
        "request_sent",
        ts_ms=1012.5,
        source="chat",
        chat_id="chat-1",
        detail={"attempt": 1},
    )
    event = parse_event(raw)
    assert event is not None
    assert event["stage"] == "request_sent"
    assert event["ts_ms"] == pytest.approx(1012.5)
    assert event["source"] == "chat"
    assert event["chat_id"] == "chat-1"
    assert event["detail"] == {"attempt": 1}

    for raw_bad in (
        "{",
        "",
        "[]",
        "null",
        "{}",
        '{"stage": "request_sent"}',
        "not-json",
    ):
        assert parse_event(raw_bad) is None


def test_observe_ignores_early_late_and_foreign_events_and_keeps_first_value():
    trace = TurnTrace()
    trace.observe({"stage": "request_sent", "ts_ms": 10.0, "chat_id": "chat-1"})
    assert trace.dropped_events == 1

    trace.open("chat-1", "pib-cloud", origin_ms=1000.0)
    trace.observe({"stage": "end_of_speech", "ts_ms": 999.0, "chat_id": "chat-1"})
    trace.observe({"stage": "request_sent", "ts_ms": 1500.0, "chat_id": "other-chat"})
    trace.observe({"stage": "request_sent", "ts_ms": 1200.0, "chat_id": "chat-1"})
    trace.observe({"stage": "request_sent", "ts_ms": 1800.0, "chat_id": "chat-1"})
    assert trace.elapsed("end_of_speech") is None
    assert trace.elapsed("request_sent") == pytest.approx(200.0)
    assert trace.delta("end_of_speech", "request_sent") is None
    dropped_while_open = trace.dropped_events

    record = trace.close()
    assert record is not None
    assert trace.is_open is False
    trace.observe({"stage": "first_token", "ts_ms": 1900.0, "chat_id": "chat-1"})
    assert trace.dropped_events == dropped_while_open + 1
    assert "first_token" not in record["stages"]
    assert record["stages"]["request_sent"] == pytest.approx(200.0)


def test_render_line_complete_turn_matches_the_contract_shape():
    """Numbers are the illustrative line from the PC-13 plan, hand-checked."""
    trace = TurnTrace()
    trace.open(
        "11111111-2222-3333-4444-555555555555",
        "pib-cloud",
        origin_ms=0.0,
    )
    trace.turn_id = "1f4c9a2b"
    samples = {
        "end_of_speech": 0.00,
        "text_received": 812.40,
        "request_sent": 1012.30,
        "response_headers": 1213.55,
        "first_token": 1905.62,
        "first_sentence_flushed": 1930.40,
        "first_audio": 2897.91,
        "last_audio": 4310.22,
        "turn_end": 4410.55,
        "stream_done": 1990.12,
    }
    for stage, ts_ms in samples.items():
        detail = None
        if stage == "response_headers":
            detail = {
                "upstream_ttft_ms": 880,
                "usage_prompt_tokens": 120,
                "usage_completion_tokens": 88,
                "budget_remaining_tokens": 49880,
            }
        elif stage == "stream_done":
            detail = {"stream_end": "done"}
        trace.observe(
            {
                "stage": stage,
                "ts_ms": ts_ms,
                "chat_id": "11111111-2222-3333-4444-555555555555",
                "detail": detail,
            }
        )
    line = render_line(trace.close())
    assert "\n" not in line
    assert line.startswith("[TURN_LATENCY] ")
    expected = (
        "[TURN_LATENCY] turn=1f4c9a2b "
        "chat=11111111-2222-3333-4444-555555555555 source=pib-cloud stages=8/8 "
        "end_of_speech_ms=0.00 text_received_ms=812.40 request_sent_ms=1012.30 "
        "response_headers_ms=1213.55 first_token_ms=1905.62 "
        "first_sentence_flushed_ms=1930.40 first_audio_ms=2897.91 "
        "last_audio_ms=4310.22 turn_end_ms=4410.55 stream_end=done "
        "stream_done_ms=1990.12 upstream_ttft_ms=880 usage_prompt_tokens=120 "
        "usage_completion_tokens=88 budget_remaining_tokens=49880 "
        "deltas=eos_to_text=812.40,text_to_req=199.90,req_to_headers=201.25,"
        "headers_to_first_token=692.07,first_token_to_sentence=24.78,"
        "sentence_to_first_audio=967.51,audio=1412.31,eos_to_turn_end=4410.55 "
        "missing="
    )
    assert line == expected
    for stage in EIGHT_STAGES:
        assert f"{stage}_ms=" in line


def test_render_line_names_missing_stages_and_does_not_invent_headers():
    trace = TurnTrace()
    trace.open("chat-9", "voice", origin_ms=0.0)
    trace.turn_id = "abcd1234"
    trace.observe({"stage": "first_token", "ts_ms": 50.0, "chat_id": "chat-9"})
    trace.mark_local("turn_end")
    line = render_line(trace.close())
    assert "stages=2/8" in line or "stages=1/8" in line
    assert "first_token_ms=50.00" in line
    assert "end_of_speech_ms=n/a" in line
    assert "request_sent_ms=n/a" in line
    assert "upstream_ttft_ms=" not in line
    assert "usage_prompt_tokens=" not in line
    missing = line.split("missing=", 1)[1].split()[0]
    for stage in ("end_of_speech", "request_sent", "response_headers", "first_audio"):
        assert stage in missing
    assert "first_token" not in missing.split(",")


def test_append_record_writes_jsonl_creates_parent_and_swallows_oserror(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
):
    path = tmp_path / "nested" / "turns.jsonl"
    append_record(str(path), {"turn_id": "aaaa1111", "stages": {"turn_end": 1.0}})
    append_record(str(path), {"turn_id": "bbbb2222", "stages": {}})
    lines = path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["turn_id"] == "aaaa1111"
    assert json.loads(lines[1])["turn_id"] == "bbbb2222"

    blocker = tmp_path / "not-a-directory"
    blocker.write_text("x", encoding="utf-8")
    with caplog.at_level(logging.WARNING):
        append_record(str(blocker / "turns.jsonl"), {"turn_id": "nope"})
    assert any("turn latency" in record.message for record in caplog.records)


def test_streaming_usage_frame_fills_the_usage_fields_and_headers_win():
    """A streaming turn has no X-Usage-* header; the counts arrive on the last frame."""
    trace = TurnTrace()
    trace.open("chat-1", "pib-cloud", origin_ms=1000.0)
    trace.observe(
        {
            "stage": "response_headers",
            "ts_ms": 1213.0,
            "detail": {"upstream_ttft_ms": 190, "budget_remaining_tokens": 49999.83},
        }
    )
    trace.observe(
        {
            "stage": "usage",
            "ts_ms": 1990.0,
            "detail": {"usage_prompt_tokens": 75, "usage_completion_tokens": 30},
        }
    )
    record = trace.close()
    line = render_line(record)
    assert "upstream_ttft_ms=190" in line
    assert "usage_prompt_tokens=75" in line
    assert "usage_completion_tokens=30" in line
    assert "budget_remaining_tokens=49999.83" in line

    # A non-streaming turn carries the counts as headers; those win.
    traced = TurnTrace()
    traced.open("chat-2", "pib-cloud", origin_ms=1000.0)
    traced.observe(
        {
            "stage": "response_headers",
            "ts_ms": 1100.0,
            "detail": {"usage_prompt_tokens": 11, "usage_completion_tokens": 22},
        }
    )
    traced.observe(
        {
            "stage": "usage",
            "ts_ms": 1200.0,
            "detail": {"usage_prompt_tokens": 99, "usage_completion_tokens": 98},
        }
    )
    assert "usage_prompt_tokens=11" in render_line(traced.close())
