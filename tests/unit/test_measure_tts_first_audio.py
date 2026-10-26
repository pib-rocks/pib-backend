"""Parser and statistics for tools/measure_tts_first_audio.py."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "tools" / "measure_tts_first_audio.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "measure_tts_first_audio", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = _load_module()

SAMPLE_TURN_LINE = (
    "[TURN_LATENCY] turn=1f4c9a2b "
    "chat=11111111-2222-3333-4444-555555555555 source=pib-cloud stages=8/8 "
    "end_of_speech_ms=0.00 text_received_ms=812.40 request_sent_ms=1012.30 "
    "response_headers_ms=1213.55 first_token_ms=1905.62 "
    "first_sentence_flushed_ms=1930.40 first_audio_ms=2897.91 "
    "last_audio_ms=4310.22 turn_end_ms=4410.55 stream_end=done "
    "stream_done_ms=1990.12 "
    "deltas=eos_to_text=812.40,text_to_req=199.90,req_to_headers=201.25,"
    "headers_to_first_token=692.07,first_token_to_sentence=24.78,"
    "sentence_to_first_audio=967.51,audio=1412.31,eos_to_turn_end=4410.55 "
    "missing="
)


def test_parse_perf_trace_accepts_docker_timestamp_prefix():
    raw = (
        "2026-10-14T10:15:30.123456789Z "
        "[PERF_TRACE] TTS_FIRST_AUDIO chat=chat-42 elapsed_ms=967.51"
    )
    event = mod.parse_perf_trace_line(raw)
    assert event is not None
    assert event.chat_id == "chat-42"
    assert event.elapsed_ms == pytest.approx(967.51)


def test_parse_turn_latency_extracts_stages_and_deltas():
    event = mod.parse_turn_latency_line(SAMPLE_TURN_LINE)
    assert event is not None
    assert event.chat_id == "11111111-2222-3333-4444-555555555555"
    assert event.turn_id == "1f4c9a2b"
    assert event.stages_ms["first_sentence_flushed"] == pytest.approx(1930.40)
    assert event.stages_ms["first_audio"] == pytest.approx(2897.91)
    assert event.deltas["sentence_to_first_audio"] == pytest.approx(967.51)


def test_merge_pairs_perf_with_following_turn_and_flags_missing_stages():
    perf = [
        mod.PerfTraceEvent("chat-1", 120.0, ""),
        mod.PerfTraceEvent("chat-1", 80.0, ""),
    ]
    turns = [
        mod.TurnLatencyEvent(
            "chat-1",
            "aaa",
            {"first_sentence_flushed": 10.0, "first_audio": None},
            {"sentence_to_first_audio": 50.0},
            "",
        ),
        mod.TurnLatencyEvent(
            "chat-1",
            "bbb",
            {"first_sentence_flushed": 5.0, "first_audio": 85.0},
            {
                "headers_to_first_token": 400.0,
                "first_token_to_sentence": 20.0,
                "sentence_to_first_audio": 80.0,
            },
            "",
        ),
    ]
    samples = mod.merge_turn_series(perf, turns, chat_id="chat-1")
    assert len(samples) == 2
    assert samples[0].tts_first_audio_ms == pytest.approx(120.0)
    assert "first_audio" in samples[0].missing_stages
    assert samples[1].first_audio_ms == pytest.approx(85.0)
    assert samples[1].dominating_delta == "headers_to_first_token"


def test_merge_handles_out_of_order_turn_latency_by_log_order():
    """Perf traces are authoritative; turn lines attach in log order per chat."""
    perf = [mod.PerfTraceEvent("c", 50.0, "")]
    turns = [
        mod.TurnLatencyEvent("other", "x", {}, {}, ""),
        mod.TurnLatencyEvent(
            "c",
            "y",
            {"first_sentence_flushed": 1.0, "first_audio": 51.0},
            {"sentence_to_first_audio": 50.0},
            "",
        ),
    ]
    sample = mod.merge_turn_series(perf, turns, chat_id="c")[0]
    assert sample.turn_id == "y"
    assert sample.tts_first_audio_ms == pytest.approx(50.0)


def test_summarize_series_median_p90_and_worst_dominating_delta():
    samples = [
        mod.TurnSample(1, "c", tts_first_audio_ms=100.0, dominating_delta="a"),
        mod.TurnSample(2, "c", tts_first_audio_ms=200.0, dominating_delta="b"),
        mod.TurnSample(
            3, "c", tts_first_audio_ms=300.0, dominating_delta="sentence_to_first_audio"
        ),
        mod.TurnSample(4, "c", tts_first_audio_ms=150.0, dominating_delta="c"),
    ]
    summary = mod.summarize_series(samples)
    assert summary.count == 4
    assert summary.median_ms == pytest.approx(150.0)
    assert summary.p90_ms == pytest.approx(300.0)
    assert summary.worst_ms == pytest.approx(300.0)
    assert summary.worst_index == 3
    assert summary.worst_dominating_delta == "sentence_to_first_audio"


def test_summarize_empty_series():
    summary = mod.summarize_series([])
    assert summary.count == 0
    assert summary.median_ms is None


def test_fixture_log_end_to_end():
    log = "\n".join(
        [
            SAMPLE_TURN_LINE,
            "[PERF_TRACE] TTS_FIRST_AUDIO chat=11111111-2222-3333-4444-555555555555 elapsed_ms=967.51",
            SAMPLE_TURN_LINE.replace("967.51", "500.00").replace(
                "sentence_to_first_audio=967.51", "sentence_to_first_audio=500.00"
            ),
            "[PERF_TRACE] TTS_FIRST_AUDIO chat=11111111-2222-3333-4444-555555555555 elapsed_ms=500.00",
        ]
    )
    perf, turns = mod.parse_log_lines(log.splitlines())
    samples = mod.merge_turn_series(
        perf,
        turns,
        chat_id="11111111-2222-3333-4444-555555555555",
    )
    summary = mod.summarize_series(samples)
    assert summary.count == 2
    assert summary.worst_ms == pytest.approx(967.51)
