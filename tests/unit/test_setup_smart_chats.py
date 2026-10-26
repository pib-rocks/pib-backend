"""Legacy smart-chats marker helper (runtime reads /etc/pib_smart_chats)."""

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = REPO_ROOT / "setup" / "installation_scripts" / "smart_chats.sh"


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
