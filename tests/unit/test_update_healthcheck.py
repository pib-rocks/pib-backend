"""Unit tests for the D12 health gate rule (setup/update_healthcheck.py).

The rule decides whether an update may be reported as successful: only a service that
RAN BEFORE the update and is gone afterwards fails it. Pre-existing damage is reported
instead of blocking, because the strict rule blocks the update that would fix it.
"""

from __future__ import annotations

import importlib.util
import io
import os
from contextlib import redirect_stdout
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[2] / "setup" / "update_healthcheck.py"


def _load_healthcheck():
    spec = importlib.util.spec_from_file_location("update_healthcheck", MODULE_PATH)
    if spec is None or spec.loader is None:  # pragma: no cover - import machinery
        raise RuntimeError(f"cannot load {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


healthcheck = _load_healthcheck()

STACK = ["angular-app", "flask-app", "ros-camera", "ros-motors"]


def test_a_fully_healthy_stack_has_nothing_to_report():
    result = healthcheck.classify(STACK, STACK, STACK)

    assert result["regressions"] == []
    assert result["unhealthy"] == []
    assert result["strict"] is False


def test_a_service_that_ran_before_and_is_gone_now_is_a_regression():
    result = healthcheck.classify(
        STACK, STACK, ["angular-app", "flask-app", "ros-camera"]
    )

    assert result["regressions"] == ["ros-motors"]


def test_pre_existing_damage_is_reported_but_does_not_block():
    """The measured case: ros-motors was already crash-looping before the update."""
    before = ["angular-app", "flask-app", "ros-camera"]

    result = healthcheck.classify(STACK, before, before)

    assert result["regressions"] == []
    assert result["unhealthy"] == ["ros-motors"]


def test_an_update_that_repairs_a_service_reports_no_regression():
    before = ["angular-app", "flask-app", "ros-camera"]

    result = healthcheck.classify(STACK, before, STACK)

    assert result["regressions"] == []
    assert result["unhealthy"] == []


def test_regression_and_tolerated_damage_are_reported_separately():
    before = ["angular-app", "flask-app", "ros-camera"]
    after = ["angular-app", "ros-camera"]

    result = healthcheck.classify(STACK, before, after)

    assert result["regressions"] == ["flask-app"]
    assert result["unhealthy"] == ["flask-app", "ros-motors"]


def test_nothing_running_before_falls_back_to_the_strict_rule():
    result = healthcheck.classify(STACK, [], ["angular-app"])

    assert result["strict"] is True
    assert result["regressions"] == ["flask-app", "ros-camera", "ros-motors"]


def test_services_newly_appearing_are_not_a_regression():
    result = healthcheck.classify(
        STACK + ["ros-programs"], STACK, STACK + ["ros-programs"]
    )

    assert result["regressions"] == []
    assert result["unhealthy"] == []


def test_split_services_accepts_newlines_commas_and_iterables():
    assert healthcheck.split_services("a\nb\n") == {"a", "b"}
    assert healthcheck.split_services("a,b") == {"a", "b"}
    assert healthcheck.split_services(["a", " b "]) == {"a", "b"}
    assert healthcheck.split_services("") == set()
    assert healthcheck.split_services(None) == set()


@pytest.mark.parametrize(
    "before,after,expected_exit",
    [
        (STACK, STACK, 0),
        (["flask-app"], ["flask-app"], 0),
        (["flask-app"], [], 1),
    ],
)
def test_main_reads_the_arguments_and_signals_regressions(before, after, expected_exit):
    buffer = io.StringIO()

    with redirect_stdout(buffer):
        exit_code = healthcheck.main(
            ["\n".join(STACK), "\n".join(before), "\n".join(after)]
        )

    assert exit_code == expected_exit
    assert "REGRESSIONS=" in buffer.getvalue()
    assert "UNHEALTHY=" in buffer.getvalue()


def test_main_reads_the_environment_when_no_arguments_are_given(monkeypatch):
    monkeypatch.setenv("HEALTH_EXPECTED", "\n".join(STACK))
    monkeypatch.setenv("HEALTH_BEFORE", "\n".join(STACK))
    monkeypatch.setenv("HEALTH_AFTER", "")
    buffer = io.StringIO()

    with redirect_stdout(buffer):
        exit_code = healthcheck.main([])

    output = buffer.getvalue()
    assert exit_code == 1
    assert "REGRESSIONS=angular-app,flask-app,ros-camera,ros-motors" in output
    assert "STRICT=0" in output


def test_the_script_runs_standalone_as_the_runner_calls_it(tmp_path):
    """The runner calls it as a script with the lists in the environment."""
    import subprocess
    import sys

    environment = dict(os.environ)
    environment.update(
        {
            "HEALTH_EXPECTED": "\n".join(STACK),
            "HEALTH_BEFORE": "angular-app\nflask-app\nros-camera",
            "HEALTH_AFTER": "angular-app\nflask-app\nros-camera",
        }
    )

    completed = subprocess.run(
        [sys.executable, str(MODULE_PATH)],
        capture_output=True,
        text=True,
        env=environment,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert "REGRESSIONS=\n" in completed.stdout
    assert "UNHEALTHY=ros-motors" in completed.stdout
