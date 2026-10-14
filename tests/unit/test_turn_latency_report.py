"""Percentiles and regression gate for the turn-latency report."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS = REPO_ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from turn_latency_report import main, nearest_rank_percentile, summarize  # noqa: E402


def test_nearest_rank_percentiles_match_hand_computed_ranks():
    """Nearest-rank: rank = ceil(percent/100 * n), 1-based into the sorted sample.

    Sample [10, 20, 30, 40, 100], n = 5.
    p50: ceil(0.50 * 5) = ceil(2.5) = 3 -> 30
    p95: ceil(0.95 * 5) = ceil(4.75) = 5 -> 100
    """
    sample = [100, 10, 40, 30, 20]
    assert nearest_rank_percentile(sample, 50) == 30
    assert nearest_rank_percentile(sample, 95) == 100

    records = [
        {"stages": {"first_audio": 10}},
        {"stages": {"first_audio": 20}},
        {"stages": {"first_audio": 30}},
        {"stages": {"first_audio": 40}},
        {"stages": {"first_audio": 100}},
    ]
    summary = summarize(records)
    assert summary["stages"]["first_audio"]["count"] == 5
    assert summary["stages"]["first_audio"]["p50"] == 30
    assert summary["stages"]["first_audio"]["p95"] == 100


def test_missing_stage_is_skipped_and_not_treated_as_zero():
    """Three real samples and one record that omits first_audio.

    Present values [100, 200, 400], n = 3.
    p50: ceil(0.50 * 3) = ceil(1.5) = 2 -> 200
    p95: ceil(0.95 * 3) = ceil(2.85) = 3 -> 400
    Treating the hole as 0 ms would sort [0, 100, 200, 400] and yield p50 = 100
    (rank ceil(0.50 * 4) = 2).
    """
    records = [
        {"stages": {"first_audio": 100, "last_audio": 150}},
        {"stages": {"first_audio": 200, "last_audio": 280}},
        {"stages": {"last_audio": 999}},
        {"stages": {"first_audio": 400, "last_audio": 450}},
    ]
    summary = summarize(records)
    audio = summary["stages"]["first_audio"]
    assert audio["count"] == 3
    assert audio["skipped"] == 1
    assert audio["p50"] == 200
    assert audio["p95"] == 400
    # audio delta needs both endpoints; the third record has no first_audio.
    assert summary["deltas"]["audio"]["count"] == 3


def _write_jsonl(path: Path, values: list[float]) -> None:
    lines = [
        json.dumps({"stages": {"first_audio": value, "turn_end": value + 10}})
        for value in values
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_regression_above_ten_percent_exits_nonzero(tmp_path: Path, capsys):
    current = tmp_path / "turns.jsonl"
    baseline = tmp_path / "baseline.json"
    _write_jsonl(current, [1000, 1000, 1000])
    assert main([str(current), "--write-baseline", str(baseline)]) == 0
    saved = json.loads(baseline.read_text(encoding="utf-8"))
    assert saved["stages"]["first_audio"]["p50"] == 1000

    slower = tmp_path / "slower.jsonl"
    _write_jsonl(slower, [1200, 1200, 1200])
    code = main([str(slower), "--baseline", str(baseline), "--max-regression", "10"])
    output = capsys.readouterr().out
    assert code == 1
    assert (
        "REGRESSION stage=first_audio metric=p50 base=1000.00 now=1200.00 delta=+20.00%"
        in output
    )
    assert "metric=p95" in output


def test_five_percent_deviation_is_not_a_regression(tmp_path: Path, capsys):
    current = tmp_path / "turns.jsonl"
    baseline = tmp_path / "baseline.json"
    _write_jsonl(current, [1000, 1000])
    assert main([str(current), "--write-baseline", str(baseline)]) == 0
    modest = tmp_path / "modest.jsonl"
    _write_jsonl(modest, [1050, 1050])
    code = main([str(modest), "--baseline", str(baseline), "--max-regression", "10"])
    output = capsys.readouterr().out
    assert code == 0
    assert "REGRESSION" not in output


def test_unusable_input_exits_2(tmp_path: Path):
    missing = tmp_path / "absent.jsonl"
    assert main([str(missing)]) == 2
    garbage = tmp_path / "garbage.jsonl"
    garbage.write_text("this is not json\n", encoding="utf-8")
    assert main([str(garbage)]) == 2
    bad_baseline = tmp_path / "turns.jsonl"
    _write_jsonl(bad_baseline, [10])
    assert (
        main([str(bad_baseline), "--baseline", str(tmp_path / "no-baseline.json")]) == 2
    )
