"""Unit tests for setup/update_status_reader.py."""

from __future__ import annotations

import importlib.util
from datetime import datetime, timezone
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[2] / "setup" / "update_status_reader.py"


def _load_status_reader():
    spec = importlib.util.spec_from_file_location("update_status_reader", MODULE_PATH)
    if spec is None or spec.loader is None:  # pragma: no cover - import machinery
        raise RuntimeError(f"cannot load {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


status_reader = _load_status_reader()


@pytest.mark.parametrize("state", ["done", "failed", "rolled_back", "cancelled"])
def test_terminal_predecessors_are_not_interrupted(state):
    assert status_reader.predecessor_interrupted({"state": state}) is False


@pytest.mark.parametrize("state", ["preflight", "fetching", "building", "verifying"])
def test_non_terminal_predecessors_are_interrupted(state):
    assert status_reader.predecessor_interrupted({"state": state}) is True


@pytest.mark.parametrize("document", [None, {}, [], {"state": ""}, {"state": 1}])
def test_malformed_or_absent_status_is_not_called_interrupted(document):
    assert status_reader.predecessor_interrupted(document) is False


def test_attempt_starts_at_one_and_increments_for_the_same_job():
    assert status_reader.next_attempt(None, "job-1") == 1
    assert status_reader.next_attempt({"jobId": "job-1"}, "job-1") == 2
    assert status_reader.next_attempt({"jobId": "job-1", "attempt": 3}, "job-1") == 4


def test_attempt_restarts_for_a_different_job():
    document = {"jobId": "old-job", "attempt": 7}

    assert status_reader.next_attempt(document, "new-job") == 1


@pytest.mark.parametrize("attempt", [0, -1, True, "2"])
def test_invalid_attempt_is_recovered_as_attempt_one(attempt):
    assert (
        status_reader.next_attempt({"jobId": "job-1", "attempt": attempt}, "job-1") == 1
    )


def test_command_line_interface_reports_predecessor_and_attempt(tmp_path, capsys):
    status_file = tmp_path / "status.json"
    status_file.write_text(
        '{"jobId":"job-1","state":"building","attempt":2}', encoding="utf-8"
    )

    assert status_reader.main([str(status_file), "job-1"]) == 0

    assert capsys.readouterr().out.splitlines() == [
        "PREDECESSOR_INTERRUPTED=true",
        "ATTEMPT=3",
        "PREDECESSOR_STATE=building",
    ]


def test_command_line_interface_tolerates_missing_status(tmp_path, capsys):
    assert status_reader.main([str(tmp_path / "missing.json"), "job-1"]) == 0

    assert capsys.readouterr().out.splitlines() == [
        "PREDECESSOR_INTERRUPTED=false",
        "ATTEMPT=1",
        "PREDECESSOR_STATE=unknown",
    ]


def test_attempt_at_max_blocks_the_next_start_attempt_past_max_also_blocks():
    at_limit = {"jobId": "job-1", "attempt": 3}
    past_limit = {"jobId": "job-1", "attempt": 4}

    assert status_reader.exceeded_attempt_limit(at_limit, "job-1", 3) is True
    assert status_reader.exceeded_attempt_limit(past_limit, "job-1", 3) is True


def test_attempt_below_max_does_not_block_the_next_start():
    document = {"jobId": "job-1", "attempt": 2}

    assert status_reader.exceeded_attempt_limit(document, "job-1", 3) is False


def test_attempt_limit_is_scoped_to_the_same_job():
    document = {"jobId": "old-job", "attempt": 9}

    assert status_reader.exceeded_attempt_limit(document, "new-job", 3) is False


def test_attempt_limit_tolerates_malformed_or_absent_status():
    assert status_reader.exceeded_attempt_limit(None, "job-1", 3) is False
    assert status_reader.exceeded_attempt_limit({}, "job-1", 3) is False


def test_retry_delay_remaining_is_zero_when_the_delay_has_elapsed():
    now = datetime(2026, 9, 21, 12, 0, 0, tzinfo=timezone.utc)
    updated_at = datetime(2026, 9, 21, 11, 55, 0, tzinfo=timezone.utc)

    assert status_reader.retry_delay_remaining(now, updated_at, 300) == 0
    assert status_reader.retry_delay_remaining(now, updated_at.isoformat(), 300) == 0


def test_retry_delay_remaining_counts_seconds_still_to_wait():
    now = datetime(2026, 9, 21, 12, 0, 0, tzinfo=timezone.utc)
    updated_at = datetime(2026, 9, 21, 11, 58, 0, tzinfo=timezone.utc)

    assert status_reader.retry_delay_remaining(now, updated_at, 300) == 180


def test_retry_delay_remaining_never_returns_a_negative_value():
    now = datetime(2026, 9, 21, 12, 0, 0, tzinfo=timezone.utc)
    updated_at = datetime(2026, 9, 21, 10, 0, 0, tzinfo=timezone.utc)

    assert status_reader.retry_delay_remaining(now, updated_at, 300) == 0


def test_retry_delay_remaining_is_zero_for_missing_or_malformed_timestamps():
    now = datetime(2026, 9, 21, 12, 0, 0, tzinfo=timezone.utc)

    assert status_reader.retry_delay_remaining(now, None, 300) == 0
    assert status_reader.retry_delay_remaining(now, "", 300) == 0
    assert status_reader.retry_delay_remaining(now, "not-a-timestamp", 300) == 0
    assert status_reader.retry_delay_remaining(None, now, 300) == 0


def test_command_line_interface_reports_exceeded_attempt_limit(tmp_path, capsys):
    status_file = tmp_path / "status.json"
    status_file.write_text(
        '{"jobId":"job-1","state":"building","attempt":3}', encoding="utf-8"
    )

    assert (
        status_reader.main(["exceeded_attempt_limit", str(status_file), "job-1", "3"])
        == 0
    )
    assert capsys.readouterr().out.splitlines() == ["true"]


def test_command_line_interface_reports_retry_delay_remaining(tmp_path, capsys):
    status_file = tmp_path / "status.json"
    status_file.write_text(
        '{"jobId":"job-1","updatedAt":"2020-01-01T00:00:00+00:00"}',
        encoding="utf-8",
    )

    assert status_reader.main(["retry_delay_remaining", str(status_file), "300"]) == 0
    assert capsys.readouterr().out.splitlines() == ["0"]
