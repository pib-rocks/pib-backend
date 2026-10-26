#!/usr/bin/env python3
"""Compare Direct vs Smart latency on the robot with the same model and question.

Run inside the voice container on hardware (see ``measure_turn_latency.py`` for
ROS service names). For each mode it drives one fresh chat per run (three by
default), sends the same question, and prints first-token and turn-end timings
from the turn latency JSONL (``/data/voice/turn-latency/turns.jsonl`` by default).

Ticket PR-2075 (AC1): three runs per mode, same ``assistant_model`` on both
personalities, fresh chat per run, paste the per-stage trace into Jira.

Example::

    python tools/measure_direct_vs_smart_latency.py \\
        --direct-chat-ids <uuid1> <uuid2> <uuid3> \\
        --smart-chat-ids <uuid1> <uuid2> <uuid3> \\
        --question "Wie geht es dir heute?"

Requires the key store to be unlocked and the robot network reachable.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_MEASURE = _REPO / "tools" / "measure_turn_latency.py"
DEFAULT_LOG = "/data/voice/turn-latency/turns.jsonl"
DEFAULT_RUNS = 3


def _stage_ms(record: dict, name: str) -> float | None:
    stages = record.get("stages") if isinstance(record.get("stages"), dict) else {}
    value = stages.get(name)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _summarize_record(record: dict) -> dict:
    return {
        "chat_id": record.get("chat_id"),
        "channel": record.get("channel"),
        "first_token_ms": _stage_ms(record, "first_token"),
        "turn_end_ms": _stage_ms(record, "turn_end"),
        "stages": record.get("stages"),
    }


def _run_turn(chat_id: str, question: str, log: Path, timeout: float) -> int:
    env = os.environ.copy()
    env["MEASURE_TURN_LATENCY_UTTERANCE"] = question
    cmd = [
        sys.executable,
        str(_MEASURE),
        "--chat-id",
        chat_id,
        "--turns",
        "1",
        "--log",
        str(log),
        "--timeout",
        str(timeout),
    ]
    result = subprocess.run(cmd, env=env)
    return int(result.returncode)


def _read_last_record(log: Path, chat_id: str) -> dict | None:
    if not log.is_file():
        return None
    last: dict | None = None
    with log.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                continue
            if not isinstance(item, dict):
                continue
            if item.get("chat_id") == chat_id:
                last = item
    return last


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--direct-chat-ids",
        nargs="+",
        help="One fresh Direct chat id per run (default: three placeholders).",
    )
    parser.add_argument(
        "--smart-chat-ids",
        nargs="+",
        help="One fresh Smart chat id per run (default: three placeholders).",
    )
    parser.add_argument("--question", default="Wie geht es dir heute?")
    parser.add_argument("--log", type=Path, default=Path(DEFAULT_LOG))
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args(argv)

    direct_ids = args.direct_chat_ids or [
        "DIRECT-CHAT-1",
        "DIRECT-CHAT-2",
        "DIRECT-CHAT-3",
    ]
    smart_ids = args.smart_chat_ids or ["SMART-CHAT-1", "SMART-CHAT-2", "SMART-CHAT-3"]
    if len(direct_ids) != len(smart_ids):
        print(
            "measure_direct_vs_smart_latency: direct and smart chat id lists "
            "must be the same length",
            file=sys.stderr,
        )
        return 2

    results: dict[str, list[dict]] = {"direct": [], "smart": []}
    for label, chat_ids in (("direct", direct_ids), ("smart", smart_ids)):
        for index, chat_id in enumerate(chat_ids):
            print(f"=== {label} run {index + 1}/{len(chat_ids)} chat={chat_id} ===")
            code = _run_turn(chat_id, args.question, args.log, args.timeout)
            if code != 0:
                print(
                    f"measure_direct_vs_smart_latency: turn failed mode={label}",
                    file=sys.stderr,
                )
                return code
            record = _read_last_record(args.log, chat_id)
            if record is None:
                print(
                    f"measure_direct_vs_smart_latency: no log record for chat={chat_id}",
                    file=sys.stderr,
                )
                return 1
            summary = _summarize_record(record)
            results[label].append(summary)
            print(json.dumps(summary, ensure_ascii=False))

    print(
        json.dumps(
            {"question": args.question, "runs": len(direct_ids), "results": results},
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(run())
