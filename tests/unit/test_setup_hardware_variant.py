"""Tests for the standalone setup hardware-variant resolver."""

import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
RESOLVER = REPO_ROOT / "setup" / "installation_scripts" / "resolve_hardware_variant.sh"


def resolve(*arguments: str) -> subprocess.CompletedProcess[str]:
    command = [
        "bash",
        "-uc",
        'source "$1"; shift; resolve_hardware_variant "$@"',
        "resolve-hardware-variant",
        str(RESOLVER),
        *arguments,
    ]
    return subprocess.run(command, capture_output=True, check=False, text=True)


def test_no_variant_flag_defaults_to_pib5edu():
    result = resolve()

    assert result.returncode == 0
    assert result.stdout.strip() == "pib5edu"
    assert result.stderr == ""


@pytest.mark.parametrize(
    ("flag", "variant"),
    [
        ("--pib4edu", "pib4edu"),
        ("--pib4advanced", "pib4advanced"),
        ("--pib5advanced", "pib5advanced"),
        ("--pib5museum", "pib5museum"),
    ],
)
def test_single_variant_flag_is_resolved(flag: str, variant: str):
    result = resolve(flag)

    assert result.returncode == 0
    assert result.stdout.strip() == variant
    assert result.stderr == ""


def test_multiple_variant_flags_are_rejected_with_both_names():
    result = resolve("--pib4edu", "--pib5advanced")

    assert result.returncode != 0
    assert "pib4edu" in result.stderr
    assert "pib5advanced" in result.stderr


def test_unknown_variant_flag_is_rejected():
    result = resolve("--pib6edu")

    assert result.returncode != 0
    assert "--pib6edu" in result.stderr
