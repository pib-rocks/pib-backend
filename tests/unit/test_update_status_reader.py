"""Unit tests for setup/update_status_reader.py."""

from __future__ import annotations

import importlib.util
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
