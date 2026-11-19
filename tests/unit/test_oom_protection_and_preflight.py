"""OOM protection and live-E2E memory preflight regressions (PR-2179)."""

from __future__ import annotations

import importlib.util
import os
import re
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"
RUN_ALL_TESTS = REPO_ROOT / "tests" / "run_all_tests.sh"
SYSTEM_SETTINGS = (
    REPO_ROOT / "setup" / "installation_scripts" / "set_system_settings.sh"
)
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
E2E_CONFTEST = REPO_ROOT / "tests" / "e2e" / "conftest.py"

CORE_OOM_PROTECTED_SERVICES = (
    "hermes-daemon",
    "pib-blockly-server",
    "flask-app",
    "rosbridge-ws",
    "ros-camera",
    "ros-motors",
    "ros-voice-assistant",
    "ros-programs",
    "ros-display",
    "ros-audio-io",
)

INIT_REAPING_SERVICES = (
    "hermes-daemon",
    "flask-app",
    "rosbridge-ws",
    "ros-camera",
    "ros-motors",
    "ros-voice-assistant",
    "ros-programs",
    "ros-display",
    "ros-audio-io",
)


def _extract_bash_function(script: Path, name: str) -> str:
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) ?\{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _load_e2e_conftest():
    spec = importlib.util.spec_from_file_location(
        "pib_pr2179_e2e_conftest", E2E_CONFTEST
    )
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_core_services_are_oom_protected_and_reap_zombies():
    compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding="utf-8"))
    services = compose["services"]

    for name in CORE_OOM_PROTECTED_SERVICES:
        assert (
            services[name].get("oom_score_adj") == -200
        ), f"{name} must set oom_score_adj: -200"

    for name in INIT_REAPING_SERVICES:
        assert services[name].get("init") is True, f"{name} must set init: true"


def test_configure_pipewire_oom_protection_writes_expected_dropins(tmp_path: Path):
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    sudo_stub = stub_bin / "sudo"
    sudo_stub.write_text(
        '#!/bin/bash\ncase "$1" in mkdir|tee) exec "$@" ;; esac\nexit 0\n',
        encoding="utf-8",
    )
    sudo_stub.chmod(0o755)
    systemctl_stub = stub_bin / "systemctl"
    systemctl_stub.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    systemctl_stub.chmod(0o755)

    systemd_root = tmp_path / "systemd"
    script = (
        'function print() { echo "[$1] ${2:-}"; }\n'
        + _extract_bash_function(SETUP_PIB, "require_nonempty")
        + f'\nsource "{SYSTEM_SETTINGS}"\n'
        + "configure_pipewire_oom_protection\n"
        + "configure_pipewire_oom_protection\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        PIB_SYSTEM_SETTINGS_DEFINE_ONLY="1",
        PIB_SYSTEMD_ROOT=str(systemd_root),
    )
    result = subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, env=env, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr

    user_mgr = systemd_root / "system" / "user@.service.d" / "99-pib-oom.conf"
    user_conf = systemd_root / "user.conf.d" / "99-pib-oom.conf"
    assert "OOMScoreAdjust=-200" in user_mgr.read_text(encoding="utf-8")
    assert "DefaultOOMScoreAdjust=200" in user_conf.read_text(encoding="utf-8")
    for unit in ("pipewire.service", "pipewire-pulse.service", "wireplumber.service"):
        dropin = systemd_root / "user" / f"{unit}.d" / "99-pib-oom.conf"
        assert "OOMScoreAdjust=-200" in dropin.read_text(encoding="utf-8")


def _run_mem_check(tmp_path: Path, meminfo_text: str, min_mb: str | None = None):
    meminfo = tmp_path / "meminfo"
    meminfo.write_text(meminfo_text, encoding="utf-8")
    script = (
        "RED=''; GREEN=''; YELLOW=''; NC=''\n"
        + _extract_bash_function(RUN_ALL_TESTS, "check_memory_headroom")
        + "\ncheck_memory_headroom\n"
    )
    env = dict(os.environ)
    env["PIB_MEMINFO_FILE"] = str(meminfo)
    if min_mb is not None:
        env["PIB_MIN_AVAILABLE_MEM_MB"] = min_mb
    else:
        env.pop("PIB_MIN_AVAILABLE_MEM_MB", None)
    return subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, env=env, text=True
    )


def test_run_all_tests_memory_preflight_fails_when_headroom_is_low(tmp_path: Path):
    meminfo = (
        "MemTotal:        8251600 kB\n"
        "MemFree:          120000 kB\n"
        "MemAvailable:     427008 kB\n"
        "SwapTotal:       2097136 kB\n"
        "SwapFree:         216064 kB\n"
    )
    result = _run_mem_check(tmp_path, meminfo)
    assert result.returncode == 1
    assert "MemAvailable=417 MiB < required 1500 MiB" in result.stderr
    assert "PIB_ROBOT_URL" in result.stderr


def test_run_all_tests_memory_preflight_passes_with_sufficient_ram(tmp_path: Path):
    meminfo = (
        "MemTotal:        8251600 kB\n"
        "MemAvailable:    3484784 kB\n"
        "SwapTotal:       2097136 kB\n"
        "SwapFree:        2097136 kB\n"
    )
    result = _run_mem_check(tmp_path, meminfo)
    assert result.returncode == 0, result.stderr
    assert "3403 MiB available" in result.stdout


def test_run_all_tests_memory_preflight_can_be_disabled(tmp_path: Path):
    meminfo = "MemAvailable:     102400 kB\n"
    result = _run_mem_check(tmp_path, meminfo, min_mb="0")
    assert result.returncode == 0
    assert "disabled" in result.stdout


def test_e2e_conftest_memory_guard_reports_clear_error_when_low(tmp_path: Path):
    conftest = _load_e2e_conftest()
    low = tmp_path / "meminfo_low"
    low.write_text(
        "MemAvailable:     512000 kB\n"
        "SwapTotal:       2097136 kB\n"
        "SwapFree:         524288 kB\n",
        encoding="utf-8",
    )
    err = conftest.check_live_memory_headroom(meminfo_path=low, min_available_mb=1500)
    assert err is not None
    assert "MemAvailable=500 MiB" in err
    assert "PIB_ROBOT_URL" in err

    ok = tmp_path / "meminfo_ok"
    ok.write_text("MemAvailable:    3500000 kB\n", encoding="utf-8")
    assert (
        conftest.check_live_memory_headroom(meminfo_path=ok, min_available_mb=1500)
        is None
    )
    assert (
        conftest.check_live_memory_headroom(meminfo_path=low, min_available_mb=0)
        is None
    )
