"""pib-marimo is not installed until the unit is active and the port answers.

setup_pib_marimo_service() is cut out of setup-pib.sh. systemctl, curl,
python3 and sudo are stubs.
"""

from __future__ import annotations

import os
import re
import shutil
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

SYSTEMCTL_STUB = """#!/bin/bash
PATH="/usr/bin:/bin"
printf 'systemctl %s\\n' "$*" >> "$STUB_LOG"
cmd="${1:-}"
case "$cmd" in
  is-active)
    if [ -f "$STUB_STATE/active" ]; then exit 0; fi
    echo activating
    exit 3
    ;;
  enable|daemon-reload)
    exit 0
    ;;
  restart)
    if [ "${STUB_RESTART_STATUS:-0}" -ne 0 ]; then
      exit "$STUB_RESTART_STATUS"
    fi
    if [ -f "$STUB_STATE/stay_down" ]; then
      exit 0
    fi
    touch "$STUB_STATE/active"
    exit 0
    ;;
  *)
    echo "unexpected systemctl command: $*" >&2
    exit 99
    ;;
esac
"""

CURL_STUB = """#!/bin/bash
printf 'curl %s\\n' "$*" >> "$STUB_LOG"
if [ -f "$STUB_STATE/http_ok" ] || [ -f "$STUB_STATE/active" ]; then
  printf '200'
  exit 0
fi
printf '000'
exit 7
"""

SLEEP_STUB = """#!/bin/bash
exit 0
"""

PYTHON_STUB = """#!/bin/bash
printf 'python3 %s\\n' "$*" >> "$STUB_LOG"
if [ "$1" = "-m" ] && [ "$2" = "marimo" ] && [ "$3" = "--version" ]; then
  if [ -f "$STUB_STATE/marimo_installed" ]; then
    echo "marimo 0.0.0-test"
    exit 0
  fi
  echo "No module named marimo" >&2
  exit 1
fi
if [ "$1" = "-m" ] && [ "$2" = "pip" ] && [ "$3" = "--version" ]; then
  echo "pip 0.0.0-test"
  exit 0
fi
if [ "$1" = "-m" ] && [ "$2" = "pip" ]; then
  if [ "${STUB_PIP_STATUS:-0}" -ne 0 ]; then
    echo "pip failed" >&2
    exit "$STUB_PIP_STATUS"
  fi
  touch "$STUB_STATE/marimo_installed"
  exit 0
fi
echo "unexpected python3 command: $*" >&2
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
    marimo_installed: bool = True,
    already_active: bool = False,
    stay_down: bool = False,
    pip_status: int = 0,
    runs: int = 1,
):
    log_path = tmp_path / "stub.log"
    log_path.touch()
    state = tmp_path / "state"
    state.mkdir()
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    unit_dir = tmp_path / "systemd"
    unit_dir.mkdir()
    for name, body in (
        ("sudo", SUDO_STUB),
        ("systemctl", SYSTEMCTL_STUB),
        ("curl", CURL_STUB),
        ("sleep", SLEEP_STUB),
        ("python3", PYTHON_STUB),
    ):
        _write_executable(stub_bin / name, body)
    # The unit invokes /usr/bin/python3. Point that path at the stub via a
    # directory the function does not use; the function calls /usr/bin/python3
    # by absolute path, so the test binds it through PIB... no, the script
    # hardcodes /usr/bin/python3. Replace it in the extracted function.
    if marimo_installed:
        (state / "marimo_installed").touch()
    if already_active:
        (state / "active").touch()
        (state / "http_ok").touch()
        shutil.copy(
            REPO_ROOT / "setup/setup_files/pib-marimo.service",
            unit_dir / "pib-marimo.service",
        )
    if stay_down:
        (state / "stay_down").touch()
    calls = []
    for index in range(1, runs + 1):
        calls.append(f"echo '--- run {index} ---'")
        calls.append("setup_pib_marimo_service")
        calls.append(f"echo rc{index}=$?")
    function = _extract_bash_function(SETUP_PIB, "setup_pib_marimo_service")
    function = function.replace("/usr/bin/python3", str(stub_bin / "python3"))
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "marimo_http_ok")
        + "\n"
        + function
        + "\n"
        + "\n".join(calls)
        + "\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}:/usr/bin:/bin",
        STUB_LOG=str(log_path),
        STUB_STATE=str(state),
        STUB_PIP_STATUS=str(pip_status),
        BACKEND_DIR=str(REPO_ROOT),
        PIB_MARIMO_UNIT=str(unit_dir / "pib-marimo.service"),
        PIB_MARIMO_NOTEBOOKS=str(tmp_path / "notebooks"),
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


def test_an_answering_service_is_left_running(tmp_path):
    result, log = _run(tmp_path, already_active=True, runs=2)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "rc1=0" in result.stdout
    assert "rc2=0" in result.stdout
    assert "already active" in result.stdout
    assert not any("systemctl restart" in line for line in log)


def test_a_unit_that_stays_activating_fails_the_step(tmp_path):
    result, _log = _run(tmp_path, stay_down=True)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "did not become active" in result.stdout


def test_a_failed_marimo_install_fails_the_step(tmp_path):
    result, log = _run(tmp_path, marimo_installed=False, pip_status=1)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "could not install marimo" in result.stdout
    assert not any("systemctl restart" in line for line in log)


def test_the_unit_edits_the_notebook_directory_without_a_swallowed_install():
    unit = (REPO_ROOT / "setup/setup_files/pib-marimo.service").read_text(
        encoding="utf-8"
    )
    text = SETUP_PIB.read_text(encoding="utf-8")
    function = _extract_bash_function(SETUP_PIB, "setup_pib_marimo_service")

    assert "/home/pib/programs/notebooks" in unit.split("ExecStart=", 1)[1]
    assert "MARIMO_SKIP_UPDATE_CHECK=1" in unit
    assert "is-active" in function
    assert "marimo_http_ok" in function
    assert "pip install --break-system-packages marimo 2>/dev/null || true" not in text
