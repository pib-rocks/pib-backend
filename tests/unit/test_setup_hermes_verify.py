"""Hermes is not installed until `hermes --version` answers.

verify_hermes_cli() is cut out of setup-pib.sh. The hermes binary and sudo
are stubs, so the test does not download the installer.
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
args=()
while [ $# -gt 0 ]; do
  case "$1" in
    -u) shift 2 ;;
    -H|-E) shift ;;
    *) args+=("$1"); shift ;;
  esac
done
exec "${args[@]}"
"""

HERMES_STUB = """#!/bin/bash
printf 'hermes %s\\n' "$*" >> "$STUB_LOG"
if [ "$1" = "--version" ]; then
  if [ -f "$STUB_STATE/version_stays_broken" ]; then
    echo "hermes: no dependency environment is committed for this install; run hermes pm repair" >&2
    exit 1
  fi
  if [ -f "$STUB_STATE/healthy" ] || [ -f "$STUB_STATE/repaired" ]; then
    echo "Hermes Agent vtest"
    exit 0
  fi
  echo "hermes: no dependency environment is committed for this install; run hermes pm repair" >&2
  exit 1
fi
if [ "$1" = "pm" ] && [ "$2" = "repair" ]; then
  if [ -f "$STUB_STATE/repair_fails" ]; then
    echo "repair failed" >&2
    exit 1
  fi
  touch "$STUB_STATE/repaired"
  exit 0
fi
echo "unexpected hermes command: $*" >&2
exit 99
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


def _run(
    tmp_path: Path,
    *,
    healthy: bool = False,
    repair_fails: bool = False,
    stays_broken: bool = False,
):
    log_path = tmp_path / "stub.log"
    log_path.touch()
    state = tmp_path / "state"
    state.mkdir()
    home = tmp_path / "home"
    binary_dir = home / ".local" / "bin"
    binary_dir.mkdir(parents=True)
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _write_executable(stub_bin / "sudo", SUDO_STUB)
    _write_executable(binary_dir / "hermes", HERMES_STUB)
    if healthy:
        (state / "healthy").touch()
    if repair_fails:
        (state / "repair_fails").touch()
    if stays_broken:
        (state / "version_stays_broken").touch()
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "hermes_as_pib")
        + "\n"
        + _extract_bash_function(SETUP_PIB, "verify_hermes_cli")
        + "\nverify_hermes_cli\necho rc=$?\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}:/usr/bin:/bin",
        HOME=str(home),
        STUB_LOG=str(log_path),
        STUB_STATE=str(state),
    )
    result = subprocess.run(
        ["/bin/bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    log = log_path.read_text(encoding="utf-8").splitlines()
    return result, log


def test_a_working_version_does_not_repair(tmp_path):
    result, log = _run(tmp_path, healthy=True)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert "Hermes Agent vtest" in result.stdout
    assert not any("pm repair" in line for line in log)


def test_a_missing_dependency_environment_is_repaired_then_checked(tmp_path):
    result, log = _run(tmp_path)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert any(
        line.endswith("hermes pm repair") or "hermes pm repair" in line for line in log
    )
    assert "Hermes Agent vtest" in result.stdout
    assert "still fails" not in result.stdout


def test_a_failed_repair_fails_the_check(tmp_path):
    result, _log = _run(tmp_path, repair_fails=True)

    assert "rc=1" in result.stdout, result.stdout + result.stderr
    assert "hermes pm repair failed" in result.stdout


def test_a_version_that_stays_broken_fails_the_check(tmp_path):
    result, _log = _run(tmp_path, stays_broken=True)

    assert "rc=1" in result.stdout, result.stdout + result.stderr
    assert "still fails after pm repair" in result.stdout


def test_an_existing_binary_is_verified_before_the_step_returns():
    text = _extract_bash_function(SETUP_PIB, "install_hermes_cli")
    already_installed = text.split("Installing Hermes CLI", 1)[0]

    assert "verify_hermes_cli" in already_installed
    assert "return 0" not in already_installed
