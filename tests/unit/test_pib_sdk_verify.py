"""Unit tests for setup/pib_sdk_verify.py."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = REPO_ROOT / "setup" / "pib_sdk_verify.py"

sys.path.insert(0, str(REPO_ROOT / "setup"))
import pib_sdk_verify  # noqa: E402


@pytest.mark.parametrize(
    "installed,required,expected",
    [
        ("0.7.4", "0.7.4", True),
        ("0.7.5", "0.7.4", True),
        ("0.8.0", "0.7.4", True),
        ("0.7.3", "0.7.4", False),
        ("0.7.3.1", "0.7.4", False),
    ],
)
def test_meets_floor(installed: str, required: str, expected: bool) -> None:
    ok, _ = pib_sdk_verify.meets_floor(installed, required)
    assert ok is expected


@pytest.mark.parametrize(
    "value",
    ["", "not-a-version", "v0.7.4", "0.7.4-beta"],
)
def test_parse_version_rejects_unparsable(value: str) -> None:
    assert pib_sdk_verify.parse_version(value) is None


def test_cli_equal_versions_exit_zero() -> None:
    result = subprocess.run(
        [sys.executable, str(HELPER), "0.7.4", "0.7.4"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert result.stderr == ""


def test_cli_below_required_exit_one() -> None:
    result = subprocess.run(
        [sys.executable, str(HELPER), "0.7.3", "0.7.4"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "below required" in result.stderr


def test_cli_unparsable_installed_exit_one() -> None:
    result = subprocess.run(
        [sys.executable, str(HELPER), "broken", "0.7.4"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "not parseable" in result.stderr
