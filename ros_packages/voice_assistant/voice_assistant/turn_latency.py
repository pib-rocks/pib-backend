"""Per-turn latency trace shared by the voice processes.

The collector is pure Python and does not import ROS. Each reporter stamps
``time.monotonic()`` milliseconds. Linux CLOCK_MONOTONIC is the same clock in
every process of one container, so the voice-assistant node can subtract its
turn origin from another process's stamp.

``end_of_speech`` is the capture layer's decision that the utterance ended.
That decision is one pause threshold of silence after the last voiced frame,
not the last voiced frame itself. The assistant may pass that earlier stamp
as ``origin_ms`` when it opens the turn, which is why a captured end of speech
renders as ``0.00``. Stamps older than the origin are dropped: they belong to
a previous turn.

Nothing here blocks, retries, or speaks. A bad event is counted and ignored.

``[PERF_TRACE] TTS_FIRST_AUDIO`` is the gap from ``first_sentence_flushed`` to
``first_audio`` for one spoken turn (milliseconds, one line per turn, includes
``chat=<id>``). The voice-assistant node logs it when the audio player reports
the first answer PCM chunk.
"""

from __future__ import annotations

import json
import logging
import os
import time
import uuid
from typing import Mapping, Optional

logger = logging.getLogger(__name__)

TURN_TRACE_TOPIC = "/voice_turn_trace"

# The eight stages the acceptance check requires on every line.
EIGHT_STAGES = (
    "end_of_speech",
    "request_sent",
    "response_headers",
    "first_token",
    "first_sentence_flushed",
    "first_audio",
    "last_audio",
    "turn_end",
)

# Recorded when they happen, but they do not count toward stages=n/8.
EXTRA_STAGES = ("text_received", "stream_done")

# The stages the audio player reports. The turn owner only accepts them once
# this turn's first answer clause has been handed to the synthesiser, because a
# signal file or a filler plays through the same player.
AUDIO_STAGES = ("first_audio", "last_audio")

# A turn's playback runs in bursts: one burst per clause, and a burst ends
# whenever the queue happens to drain between clauses. ``last_audio`` therefore
# has to be the LAST burst's end, not the first one - the answer is over when
# the final clause stopped playing. Every other stage keeps its first value.
LAST_WINS_STAGES = ("last_audio",)

# Named gaps printed in ``deltas=``. Each value is later_stage - earlier_stage.
DELTA_SPECS = (
    ("eos_to_text", "end_of_speech", "text_received"),
    ("text_to_req", "text_received", "request_sent"),
    ("req_to_headers", "request_sent", "response_headers"),
    ("headers_to_first_token", "response_headers", "first_token"),
    ("first_token_to_sentence", "first_token", "first_sentence_flushed"),
    ("sentence_to_first_audio", "first_sentence_flushed", "first_audio"),
    ("audio", "first_audio", "last_audio"),
    ("eos_to_turn_end", "end_of_speech", "turn_end"),
)

HEADER_FIELDS = (
    "upstream_ttft_ms",
    "usage_prompt_tokens",
    "usage_completion_tokens",
    "budget_remaining_tokens",
)

DEFAULT_LOG_PATH = "/data/voice/turn-latency/turns.jsonl"

# Line order. text_received sits between the capture decision and the request,
# matching the concept-page split. The eight AC stages are always emitted.
_LINE_STAGES = (
    "end_of_speech",
    "text_received",
    "request_sent",
    "response_headers",
    "first_token",
    "first_sentence_flushed",
    "first_audio",
    "last_audio",
    "turn_end",
)


def audio_event_belongs_to_turn(stage: str, sentence_flushed: bool) -> bool:
    """Whether a reported audio stage belongs to this turn's answer.

    A signal file (the start/stop beep) or a thinking filler plays through the
    same audio player while a turn is open. Only audio that begins after this
    turn's first answer clause was handed to the synthesiser is the answer's
    audio; anything earlier would make ``first_audio`` - and with it the
    first-audio latency the concept page asks for - mean something else.
    """
    if stage not in AUDIO_STAGES:
        return True
    return bool(sentence_flushed)


def now_ms() -> float:
    """Monotonic milliseconds. Comparable across processes on one machine."""
    return time.monotonic() * 1000.0


def tts_first_audio_elapsed_ms(trace: TurnTrace) -> Optional[float]:
    """Milliseconds from first_sentence_flushed to first_audio, if both exist."""
    return trace.delta("first_sentence_flushed", "first_audio")


def format_perf_trace_tts_first_audio(chat_id: str, elapsed_ms: float) -> str:
    """One ``[PERF_TRACE]`` line for TTS time-to-first-sample."""
    safe_id = chat_id or ""
    return (
        f"[PERF_TRACE] TTS_FIRST_AUDIO chat={safe_id} "
        f"elapsed_ms={float(elapsed_ms):.2f}"
    )


def default_log_path() -> str:
    """Host-mounted JSONL path. ``PIB_TURN_LATENCY_LOG`` overrides the default."""
    configured = os.environ.get("PIB_TURN_LATENCY_LOG", "").strip()
    if configured:
        return configured
    return DEFAULT_LOG_PATH


def format_event(
    stage: str,
    *,
    ts_ms: Optional[float] = None,
    source: str = "",
    chat_id: str = "",
    detail: Optional[Mapping] = None,
) -> str:
    """One JSON object. ``detail`` is omitted when empty so headers are not invented."""
    payload: dict = {
        "stage": stage,
        "ts_ms": now_ms() if ts_ms is None else float(ts_ms),
        "source": source or "",
        "chat_id": chat_id or "",
    }
    if detail:
        payload["detail"] = {str(key): value for key, value in detail.items()}
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def parse_event(raw: str) -> Optional[dict]:
    """Return the event dict, or None when the payload is not a usable event."""
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    stage = data.get("stage")
    if not isinstance(stage, str) or not stage:
        return None
    try:
        ts_ms = float(data["ts_ms"])
    except (KeyError, TypeError, ValueError):
        return None
    data["stage"] = stage
    data["ts_ms"] = ts_ms
    chat_id = data.get("chat_id")
    data["chat_id"] = chat_id if isinstance(chat_id, str) else ""
    source = data.get("source")
    data["source"] = source if isinstance(source, str) else ""
    detail = data.get("detail")
    if detail is not None and not isinstance(detail, dict):
        data["detail"] = None
    return data


class TurnTrace:
    """One open turn. The first stamp for a stage wins; later copies are ignored."""

    def __init__(self) -> None:
        self.turn_id = ""
        self.chat_id = ""
        self.source = ""
        self.origin_ms = 0.0
        self.dropped_events = 0
        self.incomplete = 0
        self._open = False
        self._stages: dict[str, float] = {}
        self._details: dict[str, dict] = {}

    @property
    def is_open(self) -> bool:
        return self._open

    def open(
        self,
        chat_id: str,
        source: str,
        *,
        origin_ms: Optional[float] = None,
    ) -> None:
        """Start a turn. Origin defaults to now; pass an earlier monotonic stamp to rebase."""
        self._open = True
        self.turn_id = uuid.uuid4().hex[:8]
        self.chat_id = chat_id or ""
        self.source = source or ""
        self.origin_ms = now_ms() if origin_ms is None else float(origin_ms)
        self.dropped_events = 0
        self.incomplete = 0
        self._stages = {}
        self._details = {}

    def mark_local(self, stage: str, **detail) -> None:
        """Stamp a stage observed in this process. Extra keywords become detail."""
        if not self._open:
            self.dropped_events += 1
            return
        self.observe(
            {
                "stage": stage,
                "ts_ms": now_ms(),
                "source": self.source,
                "chat_id": self.chat_id,
                "detail": detail or None,
            }
        )

    def observe(self, event: dict) -> None:
        """Accept a reported stage for the open turn, or count a drop."""
        if not self._open or not isinstance(event, dict):
            self.dropped_events += 1
            return
        stage = event.get("stage")
        if not isinstance(stage, str) or not stage:
            self.dropped_events += 1
            return
        try:
            ts_ms = float(event["ts_ms"])
        except (KeyError, TypeError, ValueError):
            self.dropped_events += 1
            return
        if ts_ms < self.origin_ms:
            self.dropped_events += 1
            return
        chat_id = event.get("chat_id") or ""
        if (
            isinstance(chat_id, str)
            and chat_id
            and self.chat_id
            and chat_id != self.chat_id
        ):
            self.dropped_events += 1
            return
        if stage in self._stages and stage not in LAST_WINS_STAGES:
            return
        self._stages[stage] = ts_ms
        detail = event.get("detail")
        if isinstance(detail, dict) and detail:
            self._details[stage] = dict(detail)

    def elapsed(self, key: str) -> Optional[float]:
        """Milliseconds since the turn origin, or None when the stage is absent."""
        ts_ms = self._stages.get(key)
        if ts_ms is None:
            return None
        return ts_ms - self.origin_ms

    def delta(self, *keys: str) -> Optional[float]:
        """``ts(last) - ts(first)`` when every named stage was observed."""
        if len(keys) < 2:
            return None
        stamps = []
        for key in keys:
            ts_ms = self._stages.get(key)
            if ts_ms is None:
                return None
            stamps.append(ts_ms)
        return stamps[-1] - stamps[0]

    def close(self, *, incomplete: bool = False) -> Optional[dict]:
        """Freeze the open turn. A second close returns None."""
        if not self._open:
            return None
        if incomplete:
            self.incomplete = 1
        record = self.record()
        self._open = False
        return record

    def record(self) -> dict:
        """Elapsed stage map plus header fields the API actually sent."""
        stages = {
            stage: ts_ms - self.origin_ms for stage, ts_ms in self._stages.items()
        }
        record: dict = {
            "turn_id": self.turn_id,
            "chat_id": self.chat_id,
            "source": self.source,
            "origin_ms": self.origin_ms,
            "incomplete": self.incomplete,
            "dropped_events": self.dropped_events,
            "stages": stages,
        }
        headers = self._details.get("response_headers") or {}
        for field in HEADER_FIELDS:
            if field in headers:
                record[field] = headers[field]
        # A streaming turn carries the provider's usage report on its closing
        # frame instead of in a header (interface description), so the same two
        # fields may arrive as the ``usage`` stage. Headers win when both exist.
        usage = self._details.get("usage") or {}
        for field in HEADER_FIELDS:
            if field not in record and field in usage:
                record[field] = usage[field]
        stream_detail = self._details.get("stream_done") or {}
        stream_end = stream_detail.get("stream_end")
        if isinstance(stream_end, str) and stream_end:
            record["stream_end"] = stream_end
        return record


def _fmt_ms(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.2f}"


def _fmt_header(value: object) -> str:
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if value.is_integer():
            return str(int(value))
        return f"{value:.2f}"
    return str(value)


def render_line(record: Mapping) -> str:
    """The single ``[TURN_LATENCY]`` line. Missing AC stages are ``n/a``."""
    stages = record.get("stages") if isinstance(record.get("stages"), dict) else {}
    present = [
        stage for stage in EIGHT_STAGES if _number(stages.get(stage)) is not None
    ]
    missing = [stage for stage in EIGHT_STAGES if stage not in present]
    parts = [
        "[TURN_LATENCY]",
        f"turn={record.get('turn_id', '')}",
        f"chat={record.get('chat_id', '')}",
        f"source={record.get('source', '')}",
        f"stages={len(present)}/8",
    ]
    for stage in _LINE_STAGES:
        parts.append(f"{stage}_ms={_fmt_ms(_number(stages.get(stage)))}")
    stream_end = record.get("stream_end")
    if isinstance(stream_end, str) and stream_end:
        parts.append(f"stream_end={stream_end}")
    parts.append(f"stream_done_ms={_fmt_ms(_number(stages.get('stream_done')))}")
    if int(record.get("incomplete") or 0):
        parts.append("incomplete=1")
    for field in HEADER_FIELDS:
        if field in record and record[field] is not None:
            parts.append(f"{field}={_fmt_header(record[field])}")
    delta_tokens = []
    for name, earlier, later in DELTA_SPECS:
        start = _number(stages.get(earlier))
        end = _number(stages.get(later))
        if start is None or end is None:
            continue
        delta_tokens.append(f"{name}={end - start:.2f}")
    parts.append("deltas=" + ",".join(delta_tokens))
    parts.append("missing=" + ",".join(missing))
    return " ".join(parts)


def _number(value: object) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def append_record(path: str, record: Mapping) -> None:
    """Append one JSON object. An unwritable path logs a warning and returns."""
    try:
        payload = json.dumps(
            record,
            separators=(",", ":"),
            sort_keys=True,
            ensure_ascii=False,
            default=str,
        )
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(payload + "\n")
    except OSError as exc:
        logger.warning("turn latency record was not written: %s", exc)
