"""Unit tests for setup/update_watchdog.py."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[2] / "setup" / "update_watchdog.py"


def _load_watchdog():
    spec = importlib.util.spec_from_file_location("update_watchdog", MODULE_PATH)
    if spec is None or spec.loader is None:  # pragma: no cover - import machinery
        raise RuntimeError(f"cannot load {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


watchdog = _load_watchdog()


def test_disabled_watchdog_stays_disabled():
    assert watchdog.target_timeout_us(0, 900_000_000) == 0


def test_short_watchdog_is_extended_to_the_requested_timeout():
    assert watchdog.target_timeout_us(60_000_000, 900_000_000) == 900_000_000


def test_longer_existing_watchdog_is_never_shortened():
    assert watchdog.target_timeout_us(1_200_000_000, 900_000_000) == 1_200_000_000


def test_extension_is_capped_but_an_existing_larger_value_is_preserved():
    assert watchdog.target_timeout_us(60_000_000, 9_000_000_000) == 1_800_000_000
    assert watchdog.target_timeout_us(2_000_000_000, 9_000_000_000) == 2_000_000_000


@pytest.mark.parametrize("current,requested", [(-1, 1), (1, -1), (True, 1), (1, False)])
def test_timeout_decision_rejects_invalid_inputs(current, requested):
    with pytest.raises(ValueError):
        watchdog.target_timeout_us(current, requested)


@pytest.mark.parametrize(
    "document,expected",
    [
        ("0\n", 0),
        ("900000000", 900_000_000),
        (b"1800000000\n", 1_800_000_000),
    ],
)
def test_target_document_accepts_only_values_the_helper_can_apply(document, expected):
    assert watchdog.validate_target_document(document) == expected


@pytest.mark.parametrize(
    "document",
    ["", "-1", "1 2", "1 ", "1800000001", "1min", b"\xff"],
)
def test_target_document_rejects_invalid_or_excessive_values(document):
    with pytest.raises(ValueError):
        watchdog.validate_target_document(document)


@pytest.mark.parametrize(
    "display,expected",
    [
        ("0", 0),
        ("1min", 60_000_000),
        ("1min 30s", 90_000_000),
        ("1.5s", 1_500_000),
        ("500ms", 500_000),
    ],
)
def test_systemd_duration_parser_handles_systemctl_output(display, expected):
    assert watchdog.parse_systemd_duration_us(display) == expected


def test_command_line_interface_prints_the_decision(capsys):
    assert watchdog.main(["target", "1min", "900000000"]) == 0
    assert capsys.readouterr().out == "900000000\n"


def test_command_line_interface_prints_the_parsed_current_value(capsys):
    assert watchdog.main(["parse", "1min 30s"]) == 0
    assert capsys.readouterr().out == "90000000\n"
