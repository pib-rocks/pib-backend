"""The host-IP step writes the file the API reads, and fails when it cannot.

setup_ip_dispatcher() is cut out of setup-pib.sh and run against a scratch
directory. `ip` and `sudo` are stubs, so the test does not touch /etc.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"

SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
"""

SUDO_STUB = """#!/bin/bash
printf 'sudo %s\\n' "$*" >> "$STUB_LOG"
exec "$@"
"""

IP_STUB = """#!/bin/bash
printf 'ip %s\\n' "$*" >> "$STUB_LOG"
if [ "${STUB_IP_FAIL:-0}" -ne 0 ]; then
  echo "network is unreachable" >&2
  exit "$STUB_IP_FAIL"
fi
echo "1.0.0.0 via 10.0.0.1 dev eth0 src 10.1.2.3 uid 0"
exit 0
"""


def _extract_bash_function(script: Path, name: str) -> str:
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) \{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _write_executable(path: Path, body: str) -> None:
    path.write_text(body, encoding="utf-8")
    path.chmod(0o755)


def _run(tmp_path: Path, *, ip_fail: int = 0, runs: int = 1):
    log_path = tmp_path / "stub.log"
    log_path.touch()
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _write_executable(stub_bin / "sudo", SUDO_STUB)
    _write_executable(stub_bin / "ip", IP_STUB)
    primary = tmp_path / "pib_host_ip"
    legacy = tmp_path / "flask" / "host_ip.txt"
    dispatcher = tmp_path / "dispatcher" / "99-update-ip.sh"
    calls = []
    for index in range(1, runs + 1):
        calls.append(f"echo '--- run {index} ---'")
        calls.append("setup_ip_dispatcher")
        calls.append(f"echo rc{index}=$?")
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "host_ip_file_ok")
        + "\n"
        + _extract_bash_function(SETUP_PIB, "setup_ip_dispatcher")
        + "\n"
        + "\n".join(calls)
        + "\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}:/usr/bin:/bin",
        STUB_LOG=str(log_path),
        STUB_IP_FAIL=str(ip_fail),
        PIB_HOST_IP_FILE=str(primary),
        PIB_HOST_IP_LEGACY_FILE=str(legacy),
        PIB_NM_DISPATCHER=str(dispatcher),
        BACKEND_DIR=str(tmp_path / "backend"),
    )
    result = subprocess.run(
        ["/bin/bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    return result, primary, legacy, dispatcher


def test_the_step_writes_both_host_ip_files(tmp_path):
    result, primary, legacy, dispatcher = _run(tmp_path, runs=2)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "rc1=0" in result.stdout
    assert "rc2=0" in result.stdout
    assert primary.read_text(encoding="utf-8") == "10.1.2.3\n"
    assert legacy.read_text(encoding="utf-8") == "10.1.2.3\n"
    script = dispatcher.read_text(encoding="utf-8")
    assert str(primary) in script
    assert str(legacy) in script
    # A second run keeps the same address and does not report a missing file.
    assert "was not written" not in result.stdout


def test_a_missing_address_fails_the_step(tmp_path):
    result, primary, legacy, _dispatcher = _run(tmp_path, ip_fail=1)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "was not written" in result.stdout
    assert not primary.exists()
    assert not legacy.exists()


def test_compose_mounts_the_file_the_step_writes():
    compose = (REPO_ROOT / "docker-compose.yaml").read_text(encoding="utf-8")
    config = (REPO_ROOT / "pib_api/flask/config.py").read_text(encoding="utf-8")

    assert "HOST_IP_FILE=/etc/pib_host_ip" in compose
    assert "/etc/pib_host_ip:/etc/pib_host_ip:ro" in compose
    assert 'os.getenv("HOST_IP_FILE"' in config
    text = SETUP_PIB.read_text(encoding="utf-8")
    assert "PIB_HOST_IP_FILE:-/etc/pib_host_ip" in text
    assert (
        "abort_setup"
        in text.split('run_step "Set up IP dispatcher"', 1)[1].split("\n", 1)[0]
    )
