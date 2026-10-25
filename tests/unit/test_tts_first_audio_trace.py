"""[PERF_TRACE] TTS_FIRST_AUDIO line format (PR-2054)."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from voice_assistant.turn_latency import (  # noqa: E402
    TurnTrace,
    format_perf_trace_tts_first_audio,
    tts_first_audio_elapsed_ms,
)


def test_format_matches_perf_trace_convention():
    line = format_perf_trace_tts_first_audio("chat-42", 967.5123)
    assert line == "[PERF_TRACE] TTS_FIRST_AUDIO chat=chat-42 elapsed_ms=967.51"
    assert re.fullmatch(
        r"\[PERF_TRACE\] TTS_FIRST_AUDIO chat=\S+ elapsed_ms=\d+\.\d{2}", line
    )


def test_elapsed_ms_from_turn_trace_stages():
    trace = TurnTrace()
    trace.open("chat-1", "hermes", origin_ms=1000.0)
    trace.observe(
        {
            "stage": "first_sentence_flushed",
            "ts_ms": 1000.0,
            "source": "hermes",
            "chat_id": "chat-1",
        }
    )
    trace.observe(
        {
            "stage": "first_audio",
            "ts_ms": 1967.5,
            "source": "audio_player",
            "chat_id": "chat-1",
        }
    )
    assert tts_first_audio_elapsed_ms(trace) == 967.5
