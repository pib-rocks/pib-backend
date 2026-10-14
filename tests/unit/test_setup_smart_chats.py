"""Installer flag --no-smart-chats writes the marker the runtime reads."""

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = REPO_ROOT / "setup" / "installation_scripts" / "smart_chats.sh"
SETUP = REPO_ROOT / "setup" / "setup-pib.sh"


def marker(enabled: str) -> str:
    result = subprocess.run(
        [
            "bash",
            "-uc",
            'source "$1"; smart_chats_marker "$2"',
            "smart-chats-marker",
            str(HELPER),
            enabled,
        ],
        capture_output=True,
        check=False,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


def test_marker_text():
    assert marker("1") == "enabled\n"
    assert marker("0") == "disabled\n"


def test_setup_script_records_the_flag():
    text = SETUP.read_text(encoding="utf-8")
    assert "--no-smart-chats)" in text
    assert "/etc/pib_smart_chats" in text
    assert "PIB_SMART_CHATS" in text
