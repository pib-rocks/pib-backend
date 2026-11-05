"""Voice health probe gates fresh setup and update verification (PR-2205).

Reads setup shell sources as text; one subprocess runs voice_healthcheck.sh with a
non-existent container name to assert the skip path (rc=0).
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
UPDATE_RUNNER = REPO_ROOT / "setup" / "update_runner.sh"
VOICE_HEALTH = REPO_ROOT / "setup" / "installation_scripts" / "voice_healthcheck.sh"


def test_voice_health_script_has_bounded_retry_defaults():
    text = VOICE_HEALTH.read_text(encoding="utf-8")
    assert "PIB_VOICE_HEALTH_ATTEMPTS" in text
    assert 'ATTEMPTS="${PIB_VOICE_HEALTH_ATTEMPTS:-8}"' in text
    assert re.search(r"\bwhile\b|\bfor\b", text), "expected a retry loop"
    assert "PIB_VOICE_HEALTH_INTERVAL_SECONDS" in text


def test_setup_pib_runs_voice_healthcheck_in_run_step():
    text = SETUP_PIB.read_text(encoding="utf-8")
    assert re.search(
        r'run_step "Verify voice assistant"\s+bash\s+"\$SETUP_INSTALLATION_DIR/voice_healthcheck\.sh"',
        text,
    )


def test_update_runner_snapshots_voice_health_and_gates_in_verify_result():
    text = UPDATE_RUNNER.read_text(encoding="utf-8")
    assert "VOICE_HEALTH_BEFORE" in text
    assert "docker inspect" in text and "State.Health.Status" in text
    verify = re.search(
        r"verify_result\(\) \{(.*?)^\}",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert verify, "verify_result not found"
    body = verify.group(1)
    assert "installation_scripts/voice_healthcheck.sh" in body
    assert "healthy before the update" in body
    assert "unhealthy before and after the update (D12)" in body
    # Bounded inside verify_result: wait_for_stacks already waited for the stack.
    assert "PIB_VOICE_HEALTH_ATTEMPTS=3" in body


def test_voice_health_exits_zero_when_container_missing():
    result = subprocess.run(
        ["bash", str(VOICE_HEALTH)],
        capture_output=True,
        check=False,
        text=True,
        env={**os.environ, "PIB_VOICE_CONTAINER": "nonexistent-container-pr2205"},
    )
    assert result.returncode == 0
    assert "skipping" in result.stdout.lower()


def test_voice_health_skip_before_unconditional_failure():
    text = VOICE_HEALTH.read_text(encoding="utf-8")
    skip_idx = text.find("exit 0")
    fail_idx = text.find("exit 1")
    assert skip_idx != -1 and fail_idx != -1
    assert skip_idx < fail_idx, "skip path must come before exit 1"


def test_setup_scripts_use_sudo_docker_fallback_and_respeaker_profile_3():
    """Fresh setup-pib.sh runs before relogin, so docker commands need sudo -n docker fallback."""
    install_model = (
        REPO_ROOT / "setup" / "installation_scripts" / "install_robot_model.sh"
    ).read_text(encoding="utf-8")
    voice_health = VOICE_HEALTH.read_text(encoding="utf-8")
    setup_pib = SETUP_PIB.read_text(encoding="utf-8")
    compose = (REPO_ROOT / "docker-compose.yaml").read_text(encoding="utf-8")
    assistant = (
        REPO_ROOT
        / "ros_packages"
        / "voice_assistant"
        / "voice_assistant"
        / "assistant.py"
    ).read_text(encoding="utf-8")

    assert "docker_cmd()" in install_model
    assert "sudo -n docker" in install_model
    assert "docker_cmd()" in voice_health
    assert "sudo -n docker" in voice_health
    assert "wpctl set-profile" in setup_pib
    assert "PIB_CLOUD_LIVE_SESSION=${PIB_CLOUD_LIVE_SESSION:-1}" in compose
    assert (
        "PIB_CLOUD_SESSION_TIMEOUT_SECONDS=${PIB_CLOUD_SESSION_TIMEOUT_SECONDS:-20}"
        in compose
    )
    assert "PIB_DISPLAY_PASSWORD_PROMPT=0" in compose
    assert '"PIB_CLOUD_LIVE_SESSION", "1"' in assistant
