"""Installer flag --direct-llm yields a Hermes-free host setup (PR-2065)."""

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER = REPO_ROOT / "setup" / "installation_scripts" / "smart_chats.sh"
SETUP = REPO_ROOT / "setup" / "setup-pib.sh"
DOCKER_INSTALL = REPO_ROOT / "setup" / "installation_scripts" / "docker_install.sh"
COMPOSE = REPO_ROOT / "docker-compose.yaml"


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


def _main_setup_body() -> str:
    return SETUP.read_text(encoding="utf-8").split(
        "# ---------- SETUP STARTS FROM HERE -----------", 1
    )[1]


def test_setup_parses_direct_llm_and_not_no_smart_chats():
    text = SETUP.read_text(encoding="utf-8")
    assert "--direct-llm)" in text
    assert "--no-smart-chats)" not in text
    assert "/etc/pib_smart_chats" in text
    assert "PIB_SMART_CHATS" in text


def test_direct_llm_records_disabled_smart_chats_and_exports_zero():
    main = _main_setup_body()
    assert re.search(
        r"--direct-llm\)\s*\n\s*SMART_CHATS_ENABLED=0",
        main,
    )
    assert re.search(
        r'if \[ "\$SMART_CHATS_ENABLED" = "0" \]; then\s*\n\s*export PIB_SMART_CHATS=0',
        main,
    )


def test_default_install_exports_smart_chats_enabled():
    main = _main_setup_body()
    assert "SMART_CHATS_ENABLED=1" in main
    assert re.search(
        r"else\s*\n\s*export PIB_SMART_CHATS=1",
        main,
    )


def test_hermes_cli_install_runs_only_when_smart_chats_enabled():
    main = _main_setup_body()
    assert re.search(
        r'if \[ "\$\{PIB_SMART_CHATS:-1\}" != "0" \]; then\s*\n'
        r'\s*run_step "Install Hermes CLI" install_hermes_cli',
        main,
    )
    assert 'run_step "Install Hermes CLI" install_hermes_cli' in main
    # Must not run the Hermes installer unconditionally at the top level.
    assert main.count('run_step "Install Hermes CLI" install_hermes_cli') == 1


def test_compose_hermes_daemon_is_behind_hermes_profile():
    text = COMPOSE.read_text(encoding="utf-8")
    daemon = text.split("hermes-daemon:", 1)[1].split("\n  pib-blockly-server:", 1)[0]
    assert "profiles:" in daemon
    assert "hermes" in daemon


def test_docker_install_compose_profiles_include_hermes_only_in_smart_mode():
    text = DOCKER_INSTALL.read_text(encoding="utf-8")
    assert "function backend_compose_profiles()" in text
    assert re.search(
        r'if \[ "\$\{PIB_SMART_CHATS:-1\}" != "0" \]; then\s*\n\s*profiles\+\=\(hermes\)',
        text,
    )
    start = text.split("function start_container()", 1)[1]
    assert "backend_compose_profiles" in start
    assert "hermes-daemon" not in start


def test_docker_install_skips_hermes_view_when_direct_llm():
    text = DOCKER_INSTALL.read_text(encoding="utf-8")
    tail = text.split('run_step "Install Docker Engine"', 1)[1]
    assert re.search(
        r'if \[ "\$\{PIB_SMART_CHATS:-1\}" != "0" \]; then\s*\n'
        r'\s*run_step "Set up Hermes view service" setup_hermes_view_service',
        tail,
    )


def test_backend_compose_profiles_shell_harness():
    script = """
source "$1"
backend_compose_profiles
"""

    def profiles(smart_chats: str) -> list[str]:
        result = subprocess.run(
            [
                "bash",
                "-uc",
                script,
                "backend-compose-profiles",
                str(DOCKER_INSTALL),
            ],
            capture_output=True,
            check=False,
            text=True,
            env={**os.environ, "PIB_SMART_CHATS": smart_chats},
        )
        assert result.returncode == 0, result.stderr
        return [line for line in result.stdout.splitlines() if line]

    assert profiles("0") == ["all"]
    assert profiles("1") == ["all", "hermes"]
