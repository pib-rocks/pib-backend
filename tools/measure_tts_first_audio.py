#!/usr/bin/env python3
"""Spoken-turn series for ``[PERF_TRACE] TTS_FIRST_AUDIO`` on a live robot.

Drives typed chat turns through ``tools/measure_turn_latency.py`` inside the
voice container (``send_chat_message`` → streaming answer → TTS → playback), then
parses ``TTS_FIRST_AUDIO`` and the surrounding ``[TURN_LATENCY]`` stages from
container logs.

Read-only on the robot: no restart, deploy, or config change. Log collection
uses ``docker logs`` over SSH when the JSONL file is not mounted on the host.

Audio-file injection is not wired here: the player service only plays WAV paths
inside the container and does not complete an assistant turn by itself. For
microphone-driven turns, use ``--observe-only`` and speak while the tool tails
logs.

Example (automated drive)::

    python tools/measure_tts_first_audio.py \\
        --host 192.168.1.172 \\
        --chat-id b4f01552-0c09-401c-8fde-fda753fb0261 \\
        --personality Eva \\
        --turns 5

Example (parse a saved log)::

    python tools/measure_tts_first_audio.py --log-file voice.log --turns 5
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable, Optional

_REPO = Path(__file__).resolve().parents[1]
_TOOLS = _REPO / "tools"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from turn_latency_report import nearest_rank_percentile  # noqa: E402

DEFAULT_VOICE_CONTAINER = "multirepo-ros-voice-assistant-1"
DEFAULT_HARNESS = "/app/ros2_ws/tools/measure_turn_latency.py"
BUDGET_MS = 600.0

PERF_TRACE_RE = re.compile(
    r"\[PERF_TRACE\] TTS_FIRST_AUDIO chat=(\S+) elapsed_ms=([\d.]+)"
)
TURN_LATENCY_PREFIX = "[TURN_LATENCY]"
STAGE_MS_RE = re.compile(r"(\w+)_ms=([\d.]+|n/a)")
DELTAS_RE = re.compile(r"deltas=([^ ]+)")
CHAT_RE = re.compile(r"chat=(\S+)")
TURN_RE = re.compile(r"turn=(\S+)")

PATH_TO_FIRST_AUDIO = (
    "eos_to_text",
    "text_to_req",
    "req_to_headers",
    "headers_to_first_token",
    "first_token_to_sentence",
    "sentence_to_first_audio",
)

DEFAULT_PROMPTS = (
    "Hallo.",
    "Wie geht es dir heute?",
    "Bitte erzähl mir in zwei Sätzen, was du kannst.",
)


@dataclass
class PerfTraceEvent:
    chat_id: str
    elapsed_ms: float
    line: str


@dataclass
class TurnLatencyEvent:
    chat_id: str
    turn_id: str
    stages_ms: dict[str, Optional[float]]
    deltas: dict[str, float]
    line: str


@dataclass
class TurnSample:
    index: int
    chat_id: str
    turn_id: str = ""
    first_sentence_flushed_ms: Optional[float] = None
    first_audio_ms: Optional[float] = None
    tts_first_audio_ms: Optional[float] = None
    deltas: dict[str, float] = field(default_factory=dict)
    dominating_delta: str = ""
    missing_stages: list[str] = field(default_factory=list)


@dataclass
class SeriesSummary:
    count: int
    median_ms: Optional[float]
    p90_ms: Optional[float]
    worst_ms: Optional[float]
    worst_index: Optional[int]
    worst_dominating_delta: str
    budget_ms: float = BUDGET_MS


def strip_log_prefix(line: str) -> str:
    """Drop a leading docker ``--timestamps`` ISO prefix when present."""
    text = line.strip()
    if len(text) < 32 or text[4] != "-" or text[10] != "T":
        return text
    space = text.find(" ")
    if space <= 0:
        return text
    prefix = text[:space]
    if prefix.endswith("Z") or "+" in prefix[19:] or prefix[19:20] == "-":
        return text[space + 1 :].strip()
    return text


def parse_perf_trace_line(line: str) -> Optional[PerfTraceEvent]:
    """Parse one ``[PERF_TRACE] TTS_FIRST_AUDIO`` line."""
    text = strip_log_prefix(line)
    match = PERF_TRACE_RE.search(text)
    if not match:
        return None
    return PerfTraceEvent(
        chat_id=match.group(1),
        elapsed_ms=float(match.group(2)),
        line=text,
    )


def _parse_stage_value(raw: str) -> Optional[float]:
    if raw == "n/a":
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def parse_turn_latency_line(line: str) -> Optional[TurnLatencyEvent]:
    """Parse one ``[TURN_LATENCY]`` line for stage times and named deltas."""
    text = strip_log_prefix(line)
    if TURN_LATENCY_PREFIX not in text:
        return None
    chat_match = CHAT_RE.search(text)
    turn_match = TURN_RE.search(text)
    if not chat_match:
        return None
    stages: dict[str, Optional[float]] = {}
    for stage, raw in STAGE_MS_RE.findall(text):
        stages[stage] = _parse_stage_value(raw)
    deltas: dict[str, float] = {}
    deltas_match = DELTAS_RE.search(text)
    if deltas_match:
        for token in deltas_match.group(1).split(","):
            if "=" not in token:
                continue
            name, value = token.split("=", 1)
            try:
                deltas[name] = float(value)
            except ValueError:
                continue
    return TurnLatencyEvent(
        chat_id=chat_match.group(1),
        turn_id=turn_match.group(1) if turn_match else "",
        stages_ms=stages,
        deltas=deltas,
        line=text,
    )


def parse_log_lines(
    lines: Iterable[str],
) -> tuple[list[PerfTraceEvent], list[TurnLatencyEvent]]:
    perf: list[PerfTraceEvent] = []
    turns: list[TurnLatencyEvent] = []
    for line in lines:
        event = parse_perf_trace_line(line)
        if event is not None:
            perf.append(event)
            continue
        turn = parse_turn_latency_line(line)
        if turn is not None:
            turns.append(turn)
    return perf, turns


def dominating_delta(deltas: dict[str, float]) -> str:
    """Largest pipeline segment on the path to first audio."""
    best_name = ""
    best_value = -1.0
    for name in PATH_TO_FIRST_AUDIO:
        value = deltas.get(name)
        if value is None:
            continue
        if value > best_value:
            best_value = value
            best_name = name
    if best_name:
        return best_name
    if not deltas:
        return ""
    return max(deltas.items(), key=lambda item: item[1])[0]


def merge_turn_series(
    perf_events: list[PerfTraceEvent],
    turn_events: list[TurnLatencyEvent],
    *,
    chat_id: str = "",
    expected_turns: Optional[int] = None,
) -> list[TurnSample]:
    """Pair each perf trace with the nearest following turn-latency line."""
    filtered_perf = [
        event
        for event in perf_events
        if not chat_id or event.chat_id == chat_id or chat_id == "*"
    ]
    filtered_turns = [
        event
        for event in turn_events
        if not chat_id or event.chat_id == chat_id or chat_id == "*"
    ]
    samples: list[TurnSample] = []
    turn_index = 0
    for index, perf in enumerate(filtered_perf, start=1):
        turn: Optional[TurnLatencyEvent] = None
        while turn_index < len(filtered_turns):
            candidate = filtered_turns[turn_index]
            turn_index += 1
            if candidate.chat_id == perf.chat_id or not perf.chat_id:
                turn = candidate
                break
        flushed = None
        first_audio = None
        deltas: dict[str, float] = {}
        turn_id = ""
        missing: list[str] = []
        if turn is not None:
            turn_id = turn.turn_id
            flushed = turn.stages_ms.get("first_sentence_flushed")
            first_audio = turn.stages_ms.get("first_audio")
            deltas = dict(turn.deltas)
            for stage in ("first_sentence_flushed", "first_audio"):
                if turn.stages_ms.get(stage) is None:
                    missing.append(stage)
        else:
            missing.extend(
                ("first_sentence_flushed", "first_audio", "turn_latency_line")
            )
        sample = TurnSample(
            index=index,
            chat_id=perf.chat_id,
            turn_id=turn_id,
            first_sentence_flushed_ms=flushed,
            first_audio_ms=first_audio,
            tts_first_audio_ms=perf.elapsed_ms,
            deltas=deltas,
            dominating_delta=dominating_delta(deltas),
            missing_stages=missing,
        )
        samples.append(sample)
    if expected_turns is not None and len(samples) < expected_turns:
        for index in range(len(samples) + 1, expected_turns + 1):
            samples.append(
                TurnSample(
                    index=index,
                    chat_id=chat_id or "",
                    missing_stages=["TTS_FIRST_AUDIO"],
                )
            )
    return samples


def summarize_series(samples: list[TurnSample]) -> SeriesSummary:
    values = [
        float(sample.tts_first_audio_ms)
        for sample in samples
        if sample.tts_first_audio_ms is not None
    ]
    if not values:
        return SeriesSummary(
            count=0,
            median_ms=None,
            p90_ms=None,
            worst_ms=None,
            worst_index=None,
            worst_dominating_delta="",
        )
    worst_ms = max(values)
    worst_index = next(
        index
        for index, sample in enumerate(samples, start=1)
        if sample.tts_first_audio_ms == worst_ms
    )
    worst_sample = samples[worst_index - 1]
    return SeriesSummary(
        count=len(values),
        median_ms=nearest_rank_percentile(values, 50),
        p90_ms=nearest_rank_percentile(values, 90),
        worst_ms=worst_ms,
        worst_index=worst_index,
        worst_dominating_delta=worst_sample.dominating_delta,
    )


def format_turn_line(sample: TurnSample) -> str:
    def fmt(value: Optional[float]) -> str:
        return "n/a" if value is None else f"{value:.2f}"

    parts = [
        f"turn={sample.index}",
        f"chat={sample.chat_id}",
        f"first_sentence_flushed_ms={fmt(sample.first_sentence_flushed_ms)}",
        f"first_audio_ms={fmt(sample.first_audio_ms)}",
        f"tts_first_audio_ms={fmt(sample.tts_first_audio_ms)}",
    ]
    if sample.dominating_delta:
        parts.append(f"dominating_delta={sample.dominating_delta}")
    if sample.missing_stages:
        parts.append("missing=" + ",".join(sample.missing_stages))
    return " ".join(parts)


def format_series(summary: SeriesSummary) -> str:
    if summary.count <= 0:
        return "series count=0 (no TTS_FIRST_AUDIO samples)"
    return (
        f"series count={summary.count} "
        f"median_ms={summary.median_ms:.2f} p90_ms={summary.p90_ms:.2f} "
        f"worst_ms={summary.worst_ms:.2f} worst_turn={summary.worst_index} "
        f"worst_dominating_delta={summary.worst_dominating_delta or 'n/a'} "
        f"budget_ms={summary.budget_ms:.0f}"
    )


def build_report(
    samples: list[TurnSample],
    summary: SeriesSummary,
    *,
    personality: str = "",
) -> dict:
    payload: dict = {
        "samples": [asdict(sample) for sample in samples],
        "summary": asdict(summary),
    }
    if personality:
        payload["personality"] = personality
    return payload


def _ssh_base(args: argparse.Namespace) -> list[str]:
    command = [
        "ssh",
        "-o",
        "BatchMode=yes" if not args.ssh_password else "BatchMode=no",
        "-o",
        "ConnectTimeout=25",
        "-o",
        "StrictHostKeyChecking=no",
        f"{args.ssh_user}@{args.host}",
    ]
    if args.ssh_password:
        command = ["sshpass", "-p", args.ssh_password, *command]
    return command


def _remote_command(args: argparse.Namespace, command: str) -> str:
    completed = subprocess.run(
        [*_ssh_base(args), command],
        check=False,
        capture_output=True,
        text=True,
        timeout=args.remote_timeout,
    )
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"remote command failed ({completed.returncode}): {detail}")
    return completed.stdout


def _voice_logs(args: argparse.Namespace, since: str) -> str:
    container = shlex.quote(args.voice_container)
    return _remote_command(
        args,
        f"docker logs --timestamps --since {shlex.quote(since)} {container} 2>&1",
    )


def _remote_epoch(args: argparse.Namespace) -> str:
    return _remote_command(args, "date +%s.%N").strip()


def _run_harness_turn(
    args: argparse.Namespace,
    chat_id: str,
    prompt: str,
) -> None:
    container = shlex.quote(args.voice_container)
    harness = shlex.quote(args.harness_path)
    inner = (
        f"MEASURE_TURN_LATENCY_UTTERANCE={shlex.quote(prompt)} "
        f"python3 {harness} --chat-id {shlex.quote(chat_id)} --turns 1 "
        f"--timeout {args.turn_timeout}"
    )
    _remote_command(args, f"docker exec {container} bash -lc {shlex.quote(inner)}")


def _load_prompts(args: argparse.Namespace) -> list[str]:
    prompts: list[str] = []
    if args.prompts_file is not None:
        text = args.prompts_file.read_text(encoding="utf-8")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                prompts.append(stripped)
    if args.prompt:
        prompts.extend(args.prompt)
    if not prompts:
        prompts = list(DEFAULT_PROMPTS)
    return prompts


def _collect_log_text(args: argparse.Namespace, since: str) -> str:
    if args.log_file is not None:
        return args.log_file.read_text(encoding="utf-8")
    if not args.host:
        raise ValueError("need --host or --log-file for log collection")
    return _voice_logs(args, since)


def run_series(args: argparse.Namespace) -> tuple[list[TurnSample], SeriesSummary]:
    since = ""
    if args.host and not args.log_file and not args.observe_only:
        since = _remote_epoch(args)
    elif args.host and args.observe_only:
        since = _remote_epoch(args)
        print(
            f"observe: speak {args.turns} turn(s); tailing logs since {since}",
            file=sys.stderr,
        )
        deadline = time.monotonic() + args.observe_timeout
        while time.monotonic() < deadline:
            time.sleep(args.observe_poll)
            log_text = _voice_logs(args, since)
            perf, _ = parse_log_lines(log_text.splitlines())
            filtered = [
                event
                for event in perf
                if not args.chat_id or event.chat_id == args.chat_id
            ]
            if len(filtered) >= args.turns:
                break
        else:
            log_text = _voice_logs(args, since)
            return _analyze_log(log_text, args)
    if args.host and not args.log_file and not args.observe_only:
        prompts = _load_prompts(args)
        for index in range(args.turns):
            prompt = prompts[index % len(prompts)]
            print(
                f"driving turn {index + 1}/{args.turns}: {prompt[:60]!r}",
                file=sys.stderr,
            )
            _run_harness_turn(args, args.chat_id, prompt)
        log_text = _voice_logs(args, since)
        return _analyze_log(log_text, args)
    log_text = _collect_log_text(args, since)
    return _analyze_log(log_text, args)


def _analyze_log(
    log_text: str, args: argparse.Namespace
) -> tuple[list[TurnSample], SeriesSummary]:
    perf, turns = parse_log_lines(log_text.splitlines())
    samples = merge_turn_series(
        perf,
        turns,
        chat_id=args.chat_id or "",
        expected_turns=args.turns if args.turns else None,
    )
    summary = summarize_series(samples)
    return samples, summary


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="Robot SSH host for drive/observe modes")
    parser.add_argument("--ssh-user", default="pib")
    parser.add_argument(
        "--ssh-password",
        default=os.environ.get("PIB_ROBOT_SSH_PASSWORD", ""),
    )
    parser.add_argument("--voice-container", default=DEFAULT_VOICE_CONTAINER)
    parser.add_argument("--harness-path", default=DEFAULT_HARNESS)
    parser.add_argument(
        "--chat-id",
        help="Chat UUID tied to the personality under test (required for drive mode)",
    )
    parser.add_argument(
        "--personality",
        help="Human label recorded in JSON output (resolve chat id in Cerebra)",
    )
    parser.add_argument("--turns", type=int, default=5)
    parser.add_argument("--prompt", action="append", help="Text prompt per driven turn")
    parser.add_argument("--prompts-file", type=Path)
    parser.add_argument(
        "--audio-file",
        action="append",
        help="Not automated; use --observe-only and play audio at the robot mic",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="Parse a local log capture instead of docker logs",
    )
    parser.add_argument(
        "--observe-only",
        action="store_true",
        help="Do not drive ROS; wait for operator spoken turns while tailing logs",
    )
    parser.add_argument("--observe-timeout", type=float, default=600.0)
    parser.add_argument("--observe-poll", type=float, default=2.0)
    parser.add_argument("--turn-timeout", type=float, default=120.0)
    parser.add_argument("--remote-timeout", type=float, default=180.0)
    parser.add_argument("--json", type=Path, help="Write the series report as JSON")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.turns < 1:
        print("measure_tts_first_audio: --turns must be at least 1", file=sys.stderr)
        return 2
    if args.audio_file:
        print(
            "measure_tts_first_audio: --audio-file cannot be driven from this script; "
            "use --observe-only while playing audio at the microphone",
            file=sys.stderr,
        )
        return 2
    needs_chat = args.host and not args.log_file and not args.observe_only
    if needs_chat and not args.chat_id:
        print(
            "measure_tts_first_audio: --chat-id is required to drive turns on the robot",
            file=sys.stderr,
        )
        return 2
    if not args.host and args.log_file is None:
        print(
            "measure_tts_first_audio: pass --log-file and/or --host",
            file=sys.stderr,
        )
        return 2
    try:
        samples, summary = run_series(args)
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as exc:
        print(f"measure_tts_first_audio: {exc}", file=sys.stderr)
        return 1

    for sample in samples:
        print(format_turn_line(sample))
    print(format_series(summary))

    if summary.count <= 0:
        print(
            "FINDING: no [PERF_TRACE] TTS_FIRST_AUDIO lines in the captured log "
            "(assistant may not have spoken, audio may be muted, or trace is absent)",
            file=sys.stderr,
        )
        return 3

    if args.json is not None:
        report = build_report(samples, summary, personality=args.personality or "")
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
