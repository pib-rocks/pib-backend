"""update-pib.sh applies the Ollama listen drop-in on an already-installed device.

ensure_ollama_for_update() is cut out of setup/update-pib.sh and run in a bash
subprocess. systemctl and sudo are stubs, so the test does not need a real
ollama unit and does not restart a host service.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
UPDATE_PIB = REPO_ROOT / "setup" / "update-pib.sh"
LISTEN_HELPER = REPO_ROOT / "setup" / "installation_scripts" / "ollama_listen.sh"

DROPIN_BODY = '[Service]\nEnvironment="OLLAMA_HOST=0.0.0.0:11434"\n'

SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
"""

SUDO_STUB = """#!/bin/bash
printf 'sudo %s\\n' "$*" >> "$STUB_LOG"
exec "$@"
"""

SYSTEMCTL_STUB = """#!/bin/bash
PATH="/usr/bin:/bin"
printf 'systemctl %s\\n' "$*" >> "$STUB_LOG"
cmd="${1:-}"
case "$cmd" in
  cat)
    if [ -f "$STUB_STATE/unit" ]; then exit 0; fi
    exit 1
    ;;
  is-active)
    if [ -f "$STUB_STATE/active" ]; then exit 0; fi
    exit 3
    ;;
  daemon-reload)
    exit 0
    ;;
  restart)
    if [ "${STUB_RESTART_STATUS:-0}" -ne 0 ]; then
      echo "Failed to restart ollama.service" >&2
      exit "$STUB_RESTART_STATUS"
    fi
    exit 0
    ;;
  *)
    echo "unexpected systemctl command: $*" >&2
    exit 99
    ;;
esac
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


def _logged(log: list[str], command: str) -> int:
    return sum(1 for line in log if line == command)


def _run(
    tmp_path: Path,
    *,
    unit_installed: bool = True,
    service_active: bool = True,
    restart_status: int = 0,
    runs: int = 1,
    preseeded: bool = False,
):
    log_path = tmp_path / "stub.log"
    log_path.touch()
    state = tmp_path / "state"
    state.mkdir()
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    _write_executable(stub_bin / "sudo", SUDO_STUB)
    _write_executable(stub_bin / "systemctl", SYSTEMCTL_STUB)
    if unit_installed:
        (state / "unit").touch()
    if service_active:
        (state / "active").touch()

    dropin = tmp_path / "ollama.service.d" / "override.conf"
    if preseeded:
        dropin.parent.mkdir(parents=True)
        dropin.write_text(DROPIN_BODY, encoding="utf-8")

    calls = ["set -e"]
    for index in range(1, runs + 1):
        calls.append(f"echo '--- run {index} ---'")
        calls.append("ensure_ollama_for_update")
        calls.append(f"echo rc{index}=$?")
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(UPDATE_PIB, "ensure_ollama_for_update")
        + "\n"
        + "\n".join(calls)
        + "\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=str(stub_bin),
        STUB_LOG=str(log_path),
        STUB_STATE=str(state),
        STUB_RESTART_STATUS=str(restart_status),
        PIB_OLLAMA_DROPIN=str(dropin),
        PIB_OLLAMA_LISTEN_HELPER=str(LISTEN_HELPER),
    )
    result = subprocess.run(
        ["/bin/bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    log = log_path.read_text(encoding="utf-8").splitlines()
    return result, log, dropin


def test_update_writes_the_dropin_and_restarts_an_active_unit_once(tmp_path):
    result, log, dropin = _run(tmp_path, runs=2)

    assert result.returncode == 0, result.stdout + result.stderr
    first, second = result.stdout.split("--- run 2 ---", 1)
    assert "rc1=0" in first
    assert "restarted so OLLAMA_HOST=0.0.0.0:11434 is in effect" in first
    assert "rc2=0" in second
    assert "container listen address already configured" in second
    assert "restarted so" not in second

    assert dropin.read_text(encoding="utf-8") == DROPIN_BODY
    assert _logged(log, f"sudo /usr/bin/tee {dropin}") == 1
    assert _logged(log, "systemctl daemon-reload") == 1
    assert _logged(log, "systemctl restart ollama") == 1
    assert log.index(f"sudo /usr/bin/tee {dropin}") < log.index(
        "systemctl daemon-reload"
    )
    assert log.index("systemctl daemon-reload") < log.index("systemctl restart ollama")


def test_an_already_configured_unit_is_not_rewritten_or_restarted(tmp_path):
    result, log, dropin = _run(tmp_path, preseeded=True, runs=2)
    before = dropin.read_text(encoding="utf-8")

    assert result.returncode == 0, result.stdout + result.stderr
    assert dropin.read_text(encoding="utf-8") == before == DROPIN_BODY
    assert result.stdout.count("container listen address already configured") == 2
    assert "restarted so" not in result.stdout
    assert _logged(log, f"sudo /usr/bin/tee {dropin}") == 0
    assert _logged(log, "systemctl daemon-reload") == 0
    assert _logged(log, "systemctl restart ollama") == 0
    assert not any(line.startswith("sudo ") for line in log)


def test_a_device_without_ollama_stays_untouched(tmp_path):
    result, log, dropin = _run(
        tmp_path, unit_installed=False, service_active=False, runs=2
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "rc1=0" in result.stdout
    assert "rc2=0" in result.stdout
    assert result.stdout.count("ollama: not installed; listen drop-in left unset") == 2
    assert not dropin.exists()
    assert not dropin.parent.exists()
    assert _logged(log, "systemctl cat ollama") == 2
    assert _logged(log, "systemctl restart ollama") == 0
    assert _logged(log, "systemctl daemon-reload") == 0
    assert not any(line.startswith("sudo ") for line in log)
    assert "restarted so" not in result.stdout


def test_an_inactive_unit_gets_the_dropin_without_being_started(tmp_path):
    result, log, dropin = _run(tmp_path, service_active=False)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "rc1=0" in result.stdout
    assert "service is not active, so it was not restarted" in result.stdout
    assert dropin.read_text(encoding="utf-8") == DROPIN_BODY
    assert _logged(log, f"sudo /usr/bin/tee {dropin}") == 1
    assert _logged(log, "systemctl restart ollama") == 0


def test_a_failed_restart_fails_the_update_step(tmp_path):
    result, log, dropin = _run(tmp_path, restart_status=1)

    assert result.returncode != 0
    assert "rc1=0" not in result.stdout
    assert "in effect" not in result.stdout
    assert dropin.read_text(encoding="utf-8") == DROPIN_BODY
    assert _logged(log, "systemctl restart ollama") == 1


def test_update_backend_applies_the_dropin_after_pull_and_before_compose():
    text = UPDATE_PIB.read_text(encoding="utf-8")
    backend = text.split("function update_backend()", 1)[1].split(
        "function update_frontend()", 1
    )[0]
    helper = LISTEN_HELPER.read_text(encoding="utf-8")

    assert "ensure_ollama_for_update" in backend
    assert backend.index("git pull --ff-only origin main") < backend.index(
        "ensure_ollama_for_update"
    )
    assert backend.index("ensure_ollama_for_update") < backend.index(
        "docker compose --profile all"
    )
    assert 'Environment="OLLAMA_HOST=0.0.0.0:11434"' in helper
    assert "systemctl restart ollama" in helper
    assert "ensure_ollama_listen_dropin restart" in text
