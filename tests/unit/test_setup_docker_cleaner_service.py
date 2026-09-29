"""Tests for the docker_cleaner.service installation in docker_install.sh.

The script is executed for real in a bash subprocess; `sudo` is replaced by a
stub on PATH that records its argv and can fail `systemctl start` on demand.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKER_INSTALL = REPO_ROOT / "setup" / "installation_scripts" / "docker_install.sh"

SUCCESS_TEXT = "Docker container cleanup service installed and started"
USERMOD_CALL = "usermod -aG docker pib"
START_CALL = "systemctl start docker_cleaner.service"

# Records every sudo invocation, one command per line, with the leading
# VAR=value assignments dropped so the log holds the command that was run.
SUDO_STUB = """#!/bin/bash
args=()
for arg in "$@"; do
    if [ ${#args[@]} -eq 0 ] && [[ "$arg" == [A-Za-z_]*=* ]]; then
        continue
    fi
    args+=("$arg")
done
printf '%s\\n' "${args[*]}" >> "$STUB_LOG"

if [ "${args[0]:-}" = "systemctl" ] && [ "${args[1]:-}" = "start" ]; then
    status=${STUB_SYSTEMCTL_START_STATUS:-0}
    if [ "$status" -ne 0 ]; then
        echo "Job for docker_cleaner.service failed because the control" \\
            "process exited with error code." >&2
    fi
    exit "$status"
fi
exit 0
"""

# `command_exists docker` must be true so install_docker_engine skips the apt work.
DOCKER_STUB = """#!/bin/bash
exit 0
"""

# print() and command_exists() come from setup-pib.sh, which sources
# docker_install.sh; sourcing setup-pib.sh itself would run the installer.
HARNESS = """
function print() {
    if [ -z "${2:-}" ]; then
        echo "[[ $1 ]]"
    else
        echo "[$1][[ $2 ]]"
    fi
}
function command_exists() { command -v "$@" >/dev/null 2>&1; }
source "$DOCKER_INSTALL"
"""


def _make_stub_bin(tmp_path: Path) -> Path:
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    for name, body in (("sudo", SUDO_STUB), ("docker", DOCKER_STUB)):
        stub = stub_bin / name
        stub.write_text(body, encoding="utf-8")
        stub.chmod(0o755)
    return stub_bin


def _make_backend_dir(tmp_path: Path) -> Path:
    backend_dir = tmp_path / "pib-backend"
    blockly = (
        backend_dir
        / "pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks"
        / "custom-blocks.ts"
    )
    blockly.parent.mkdir(parents=True)
    blockly.write_text("// vendored\n", encoding="utf-8")
    (backend_dir / "pib_api/flask").mkdir(parents=True)
    return backend_dir


def _run(tmp_path: Path, script: str, start_status: int = 0):
    log = tmp_path / "sudo.log"
    log.touch()
    stub_bin = _make_stub_bin(tmp_path)
    backend_dir = _make_backend_dir(tmp_path)

    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        STUB_SYSTEMCTL_START_STATUS=str(start_status),
        BACKEND_DIR=str(backend_dir),
        FRONTEND_DIR=str(tmp_path / "cerebra"),
        DOCKER_INSTALL=str(DOCKER_INSTALL),
    )
    result = subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def _index_of(calls: list[str], needle: str) -> int:
    matches = [i for i, call in enumerate(calls) if needle in call]
    assert matches, f"{needle!r} was never invoked; recorded calls: {calls}"
    return matches[0]


def test_pib_joins_docker_group_before_the_cleaner_service_is_started(tmp_path):
    """The unit runs as User=pib, so it can only reach the socket after usermod."""
    result, calls = _run(tmp_path, HARNESS)

    assert _index_of(calls, USERMOD_CALL) < _index_of(calls, START_CALL)
    assert SUCCESS_TEXT in result.stdout


def test_failing_start_returns_non_zero_and_does_not_report_success(tmp_path):
    script = HARNESS + """
setup_docker_cleaner_service
echo "cleaner_status=$?"
"""
    result, _ = _run(tmp_path, script, start_status=1)

    assert "cleaner_status=0" not in result.stdout
    assert "cleaner_status=1" in result.stdout
    assert SUCCESS_TEXT not in result.stdout
    assert "failed to start docker_cleaner.service" in result.stdout


def test_successful_start_reports_success_and_returns_zero(tmp_path):
    script = HARNESS + """
setup_docker_cleaner_service
echo "cleaner_status=$?"
"""
    result, calls = _run(tmp_path, script)

    assert "cleaner_status=0" in result.stdout
    assert SUCCESS_TEXT in result.stdout
    assert _index_of(calls, START_CALL) >= 0
