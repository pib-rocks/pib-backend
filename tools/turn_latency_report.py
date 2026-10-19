#!/usr/bin/env python3
"""Summarise turn-latency JSONL and compare it with a baseline.

Percentiles use the NIST nearest-rank method: sort the sample ascending and
take the value at rank ``ceil(percent/100 * n)`` (1-based). A stage that is
absent from a record is skipped and counted. It is never filled in as 0 ms.

Exit codes: 0 when no metric regresses, 1 when at least one REGRESSION line
was printed, 2 when the input or the baseline cannot be used.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Iterable, Optional

_REPO = Path(__file__).resolve().parents[1]
_VOICE = _REPO / "ros_packages" / "voice_assistant"
if str(_VOICE) not in sys.path:
    sys.path.insert(0, str(_VOICE))

from voice_assistant.turn_latency import DELTA_SPECS, EIGHT_STAGES  # noqa: E402

REPORT_STAGES = EIGHT_STAGES + ("text_received", "stream_done")


def nearest_rank_percentile(values: Iterable[float], percent: float) -> float:
    """NIST nearest-rank percentile of ``values`` (order does not matter)."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("nearest-rank percentile needs at least one value")
    if percent <= 0:
        return ordered[0]
    if percent >= 100:
        return ordered[-1]
    rank = math.ceil((percent / 100.0) * len(ordered))
    rank = min(max(rank, 1), len(ordered))
    return ordered[rank - 1]


def _as_float(value: object) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def summarize(records: list[dict]) -> dict:
    """Count, skipped, p50 and p95 per stage and per named delta."""
    stage_values: dict[str, list[float]] = {name: [] for name in REPORT_STAGES}
    stage_skipped: dict[str, int] = {name: 0 for name in REPORT_STAGES}
    delta_values: dict[str, list[float]] = {name: [] for name, _, _ in DELTA_SPECS}
    delta_skipped: dict[str, int] = {name: 0 for name, _, _ in DELTA_SPECS}

    for record in records:
        stages = record.get("stages") if isinstance(record.get("stages"), dict) else {}
        for name in REPORT_STAGES:
            value = _as_float(stages.get(name))
            if value is None:
                stage_skipped[name] += 1
            else:
                stage_values[name].append(value)
        for name, earlier, later in DELTA_SPECS:
            start = _as_float(stages.get(earlier))
            end = _as_float(stages.get(later))
            if start is None or end is None:
                delta_skipped[name] += 1
            else:
                delta_values[name].append(end - start)

    return {
        "method": "nearest-rank",
        "stages": {
            name: _metric(stage_values[name], stage_skipped[name])
            for name in REPORT_STAGES
        },
        "deltas": {
            name: _metric(delta_values[name], delta_skipped[name])
            for name, _, _ in DELTA_SPECS
        },
    }


def _metric(values: list[float], skipped: int) -> dict:
    if not values:
        return {"count": 0, "skipped": skipped, "p50": None, "p95": None}
    return {
        "count": len(values),
        "skipped": skipped,
        "p50": nearest_rank_percentile(values, 50),
        "p95": nearest_rank_percentile(values, 95),
    }


def format_summary(summary: dict) -> str:
    lines = []
    for kind, section in (("stage", "stages"), ("delta", "deltas")):
        metrics = summary.get(section) or {}
        for name, metric in metrics.items():
            count = int(metric.get("count") or 0)
            skipped = int(metric.get("skipped") or 0)
            if count <= 0 or metric.get("p50") is None or metric.get("p95") is None:
                lines.append(
                    f"{kind} {name} count={count} skipped={skipped} p50=n/a p95=n/a"
                )
                continue
            lines.append(
                f"{kind} {name} count={count} skipped={skipped} "
                f"p50={float(metric['p50']):.2f} p95={float(metric['p95']):.2f}"
            )
    return "\n".join(lines)


def regression_lines(summary: dict, baseline: dict, max_percent: float) -> list[str]:
    """Metrics more than ``max_percent`` above the baseline. Slower is larger."""
    lines = []
    for section in ("stages", "deltas"):
        current_map = summary.get(section) or {}
        base_map = baseline.get(section) or {}
        if not isinstance(base_map, dict):
            continue
        for name, base_metric in base_map.items():
            if not isinstance(base_metric, dict):
                continue
            now_metric = current_map.get(name)
            if not isinstance(now_metric, dict):
                continue
            if int(now_metric.get("count") or 0) <= 0:
                continue
            if int(base_metric.get("count") or 0) <= 0:
                continue
            for key in ("p50", "p95"):
                base = _as_float(base_metric.get(key))
                now = _as_float(now_metric.get(key))
                if base is None or now is None or base <= 0:
                    continue
                percent = (now - base) / base * 100.0
                if percent > max_percent:
                    lines.append(
                        f"REGRESSION stage={name} metric={key} "
                        f"base={base:.2f} now={now:.2f} delta=+{percent:.2f}%"
                    )
    return lines


def load_records(path: Path) -> list[dict]:
    """JSON objects, one per line. A file with no usable object is an error."""
    if not path.is_file():
        raise ValueError(f"turn latency log not found: {path}")
    records: list[dict] = []
    raw = path.read_text(encoding="utf-8")
    saw_payload = False
    for line in raw.splitlines():
        text = line.strip()
        if not text:
            continue
        saw_payload = True
        try:
            item = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"turn latency log is not JSONL: {path}") from exc
        if isinstance(item, dict):
            records.append(item)
    if not saw_payload or not records:
        raise ValueError(f"turn latency log has no records: {path}")
    return records


def load_baseline(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"baseline not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"baseline is not JSON: {path}") from exc
    if not isinstance(data, dict) or "stages" not in data:
        raise ValueError(f"baseline has no stages: {path}")
    return data


def _write_baseline(path: Path, summary: dict) -> None:
    payload = {
        "method": "nearest-rank",
        "stages": summary["stages"],
        "deltas": summary["deltas"],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path, help="JSONL written by append_record")
    parser.add_argument(
        "--baseline",
        type=Path,
        default=None,
        help="Baseline JSON previously written by --write-baseline",
    )
    parser.add_argument(
        "--write-baseline",
        type=Path,
        default=None,
        help="Write the measured p50/p95 baseline to this path",
    )
    parser.add_argument(
        "--max-regression",
        type=float,
        default=10.0,
        help="Flag a metric this many percent above its baseline (default 10)",
    )
    args = parser.parse_args(argv)
    try:
        records = load_records(args.jsonl)
        summary = summarize(records)
    except (OSError, ValueError) as exc:
        print(f"turn latency report: {exc}", file=sys.stderr)
        return 2

    print(format_summary(summary))
    if args.write_baseline is not None:
        try:
            _write_baseline(args.write_baseline, summary)
        except OSError as exc:
            print(f"turn latency report: {exc}", file=sys.stderr)
            return 2

    if args.baseline is None:
        return 0
    try:
        baseline = load_baseline(args.baseline)
    except (OSError, ValueError) as exc:
        print(f"turn latency report: {exc}", file=sys.stderr)
        return 2
    lines = regression_lines(summary, baseline, args.max_regression)
    for line in lines:
        print(line)
    return 1 if lines else 0


if __name__ == "__main__":
    sys.exit(main())
