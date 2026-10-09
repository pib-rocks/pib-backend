"""setup-pib.sh installs a CPU-tuned local Ollama only when it is missing (PR-1922).

The step installs for the generation-5 (8 GiB) variants and skips the generation-4
variants (PR-1923). install_ollama_qwen_fast() is cut out of setup-pib.sh and run
in a bash subprocess. curl, sh, sudo, systemctl, ollama and free are stubs on PATH,
so the test does not need ollama installed and does not download a model.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
MODELFILE = REPO_ROOT / "setup" / "ollama" / "Modelfile"

# 2.2 GiB free: free -k reports kibibytes, so integer MiB is 2306867 // 1024 = 2252,
# comfortably above the 1200 MiB the step treats as the model's requirement.
AMPLE_AVAILABLE_KIB = 2_306_867
# 1 GiB available is under the Q4 weights plus the 2048-token KV cache.
LOW_AVAILABLE_KIB = 1_048_576

# pib5edu, pib5advanced and pib5museum are the generation-5 (8 GiB) variants.
INSTALLING_VARIANTS = ("pib5edu", "pib5advanced", "pib5museum")
SKIPPING_VARIANTS = ("pib4edu", "pib4advanced")

EXPECTED_MODELFILE = """\
FROM qwen2.5:1.5b
PARAMETER num_thread 4
PARAMETER num_ctx 2048
PARAMETER num_predict 256
PARAMETER temperature 0.5
PARAMETER top_p 0.9
SYSTEM "Du antwortest nur auf Deutsch. Antworte extrem knapp und direkt. Keine Füllwörter, keine Floskeln und keine Höflichkeitsformeln. Keine Einleitung und kein Abschied. Nur die gefragte Antwort."
"""

SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""

# Stubs reset PATH internally so cp/touch resolve, without exposing a host ollama
# binary to the function under test.
OLLAMA_STUB = """#!/bin/bash
PATH="/usr/bin:/bin"
printf 'ollama %s\\n' "$*" >> "$STUB_LOG"
cmd="${1:-}"
case "$cmd" in
  --version)
    echo "ollama version 0.0.0-test"
    exit 0
    ;;
  list)
    echo "NAME ID SIZE MODIFIED"
    if [ -f "$STUB_STATE/models" ]; then
      cat "$STUB_STATE/models"
    fi
    exit 0
    ;;
  pull)
    printf '%s\\n' "${2:-}" >> "$STUB_STATE/models"
    exit 0
    ;;
  create)
    printf '%s\\n' "qwen-fast:latest" >> "$STUB_STATE/models"
    exit 0
    ;;
  *)
    echo "unexpected ollama command: $*" >&2
    exit 99
    ;;
esac
"""

CURL_STUB = """#!/bin/bash
PATH="/usr/bin:/bin"
printf 'curl %s\\n' "$*" >> "$STUB_LOG"
if [[ "$*" == *"/api/tags"* ]]; then
  if [ "${STUB_TAGS_STATUS:-0}" -ne 0 ]; then
    echo "curl: (7) tags endpoint refused" >&2
    exit "$STUB_TAGS_STATUS"
  fi
  if [ -f "$STUB_STATE/tags.json" ]; then
    cat "$STUB_STATE/tags.json"
  else
    echo '{"models":[{"name":"qwen-fast:latest"},{"name":"qwen2.5:1.5b"}]}'
  fi
  exit 0
fi
if [ "${STUB_CURL_STATUS:-0}" -ne 0 ]; then
  echo "curl: (6) Could not resolve host" >&2
  exit "$STUB_CURL_STATUS"
fi
echo "echo official-ollama-installer"
exit 0
"""

SH_STUB = """#!/bin/bash
PATH="/usr/bin:/bin"
if [ "$#" -eq 0 ]; then
  printf 'sh\\n' >> "$STUB_LOG"
else
  printf 'sh %s\\n' "$*" >> "$STUB_LOG"
fi
cat >/dev/null
if [ "${STUB_SH_STATUS:-0}" -ne 0 ]; then
  exit "$STUB_SH_STATUS"
fi
if [ ! -x "$STUB_BIN/ollama" ]; then
  cp "$OLLAMA_STUB_SOURCE" "$STUB_BIN/ollama"
  chmod 755 "$STUB_BIN/ollama"
fi
exit 0
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
  is-active)
    if [ -f "$STUB_STATE/active" ]; then exit 0; fi
    exit 3
    ;;
  is-enabled)
    if [ -f "$STUB_STATE/enabled" ]; then exit 0; fi
    exit 1
    ;;
  enable)
    touch "$STUB_STATE/enabled"
    exit 0
    ;;
  start)
    if [ "${STUB_START_STATUS:-0}" -ne 0 ]; then
      echo "Failed to start ollama.service" >&2
      exit "$STUB_START_STATUS"
    fi
    touch "$STUB_STATE/active"
    exit 0
    ;;
  restart)
    echo "restart is not part of this step" >&2
    exit 99
    ;;
  show)
    echo "ollama"
    exit 0
    ;;
  daemon-reload)
    exit 0
    ;;
  *)
    echo "unexpected systemctl command: $*" >&2
    exit 99
    ;;
esac
"""

FREE_STUB = """#!/bin/bash
printf 'free %s\\n' "$*" >> "$STUB_LOG"
avail="${STUB_MEM_AVAILABLE_KIB:-2306867}"
echo "              total        used        free      shared  buff/cache   available"
echo "Mem:        4194304     1048576     1048576           0     2097152     ${avail}"
echo "Swap:       1048576           0     1048576"
exit 0
"""

SLEEP_STUB = """#!/bin/bash
exit 0
"""

ZSTD_STUB = """#!/bin/bash
exit 0
"""

APT_STUB = """#!/bin/bash
printf 'apt-get %s\\n' "$*" >> "$STUB_LOG"
exit "${STUB_APT_STATUS:-0}"
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
    ollama_installed: bool = False,
    models: list[str] | None = None,
    service_active: bool = False,
    service_enabled: bool = False,
    mem_available_kib: int = AMPLE_AVAILABLE_KIB,
    curl_status: int = 0,
    installer_status: int = 0,
    start_status: int = 0,
    apt_status: int = 0,
    tags_status: int = 0,
    tags_body: str | None = None,
    zstd_present: bool = True,
    runs: int = 1,
    backend_dir: Path | None = None,
    setup_dir: Path | None = None,
    hardware_variant: str = "pib5edu",
):
    log_path = tmp_path / "stub.log"
    log_path.touch()
    state = tmp_path / "state"
    state.mkdir()
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    ollama_source = tmp_path / "ollama-stub.sh"
    _write_executable(ollama_source, OLLAMA_STUB)
    for name, body in (
        ("curl", CURL_STUB),
        ("sh", SH_STUB),
        ("sudo", SUDO_STUB),
        ("systemctl", SYSTEMCTL_STUB),
        ("free", FREE_STUB),
        ("sleep", SLEEP_STUB),
        ("apt-get", APT_STUB),
    ):
        _write_executable(stub_bin / name, body)
    if zstd_present:
        _write_executable(stub_bin / "zstd", ZSTD_STUB)
    if ollama_installed:
        _write_executable(stub_bin / "ollama", OLLAMA_STUB)
    if models:
        (state / "models").write_text(
            "".join(f"{name}\n" for name in models), encoding="utf-8"
        )
    if service_active:
        (state / "active").touch()
    if service_enabled:
        (state / "enabled").touch()
    if tags_body is not None:
        (state / "tags.json").write_text(tags_body, encoding="utf-8")

    calls = []
    for index in range(1, runs + 1):
        calls.append(f"echo '--- run {index} ---'")
        calls.append("install_ollama_qwen_fast")
        calls.append(f"echo rc{index}=$?")
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "install_ollama_qwen_fast")
        + "\n"
        + "\n".join(calls)
        + "\n"
    )
    env = dict(os.environ)
    env.update(
        PATH=str(stub_bin),
        STUB_LOG=str(log_path),
        STUB_STATE=str(state),
        STUB_BIN=str(stub_bin),
        OLLAMA_STUB_SOURCE=str(ollama_source),
        STUB_CURL_STATUS=str(curl_status),
        STUB_SH_STATUS=str(installer_status),
        STUB_START_STATUS=str(start_status),
        STUB_APT_STATUS=str(apt_status),
        STUB_TAGS_STATUS=str(tags_status),
        STUB_MEM_AVAILABLE_KIB=str(mem_available_kib),
        PIB_OLLAMA_DROPIN=str(tmp_path / "ollama-override.conf"),
        BACKEND_DIR=str(backend_dir if backend_dir is not None else REPO_ROOT),
        SETUP_SCRIPT_DIR=str(
            setup_dir if setup_dir is not None else REPO_ROOT / "setup"
        ),
        PIB_OLLAMA_READY_ATTEMPTS="2",
        PIB_HARDWARE_VARIANT=hardware_variant,
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


def test_modelfile_pins_the_cpu_parameters_and_the_german_prompt():
    text = MODELFILE.read_text(encoding="utf-8")

    assert text == EXPECTED_MODELFILE
    assert "PARAMETER num_thread 4\n" in text
    assert "PARAMETER num_ctx 2048\n" in text
    assert "PARAMETER num_predict 256\n" in text
    assert "PARAMETER temperature 0.5\n" in text
    assert "PARAMETER top_p 0.9\n" in text
    system = text.split("SYSTEM ", 1)[1]
    assert "Deutsch" in system
    assert "knapp" in system
    assert "Füllwörter" in system
    assert "Höflichkeitsformeln" in system


def test_installs_pulls_and_creates_once_and_leaves_a_running_service_alone(tmp_path):
    result, log = _run(tmp_path, runs=2)

    assert result.returncode == 0, result.stdout + result.stderr
    first, second = result.stdout.split("--- run 2 ---", 1)
    assert "rc1=0" in first
    assert "installing from https://ollama.com/install.sh" in first
    assert "ollama: version ollama version 0.0.0-test" in first
    assert "service started" in first
    assert "service runs as user ollama" in first
    assert "2252 MiB available RAM" in first
    assert "is below the" not in first
    assert "pulling qwen2.5:1.5b" in first
    assert f"creating qwen-fast from {REPO_ROOT}/setup/ollama/Modelfile" in first

    assert "rc2=0" in second
    assert "already installed" in second
    assert "already active; leaving it running" in second
    assert "qwen2.5:1.5b already present; not pulling" in second
    assert "qwen-fast already present; not recreating" in second
    assert "installing from" not in second
    assert "pulling qwen2.5:1.5b" not in second
    assert "creating qwen-fast" not in second

    assert _logged(log, "curl -fsSL https://ollama.com/install.sh") == 1
    assert _logged(log, "sh") == 1
    assert not any(line.startswith("apt-get") for line in log)
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 1
    assert (
        _logged(log, f"ollama create qwen-fast -f {REPO_ROOT}/setup/ollama/Modelfile")
        == 1
    )
    assert _logged(log, "systemctl start ollama") == 1
    assert _logged(log, "systemctl restart ollama") == 0
    assert not any("restart" in line for line in log)


def test_a_second_run_on_an_existing_install_does_not_touch_the_model(tmp_path):
    result, log = _run(
        tmp_path,
        ollama_installed=True,
        models=["qwen2.5:1.5b", "qwen-fast:latest"],
        service_active=True,
        service_enabled=True,
    )

    assert "rc1=0" in result.stdout, result.stdout + result.stderr
    assert "already installed" in result.stdout
    assert "already active; leaving it running" in result.stdout
    assert "not pulling" in result.stdout
    assert "not recreating" in result.stdout
    assert _logged(log, "curl -fsSL https://ollama.com/install.sh") == 0
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0
    assert _logged(log, "systemctl start ollama") == 0
    assert _logged(log, "systemctl restart ollama") == 0
    assert not any(line.startswith("ollama create ") for line in log)


def test_warns_when_available_ram_is_below_the_model_requirement(tmp_path):
    result, log = _run(
        tmp_path,
        ollama_installed=True,
        service_active=True,
        service_enabled=True,
        mem_available_kib=LOW_AVAILABLE_KIB,
    )

    assert "rc1=0" in result.stdout, result.stdout + result.stderr
    warning = (
        "1024 MiB available RAM is below the 1200 MiB qwen2.5:1.5b needs "
        "(Q4 weights plus the 2048-token KV cache)"
    )
    assert warning in result.stdout
    assert result.stdout.index(warning) < result.stdout.index("pulling qwen2.5:1.5b")
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 1


def test_missing_zstd_is_installed_before_the_official_installer(tmp_path):
    result, log = _run(tmp_path, zstd_present=False)

    assert "rc1=0" in result.stdout, result.stdout + result.stderr
    assert log.index("apt-get install -y zstd") < log.index(
        "curl -fsSL https://ollama.com/install.sh"
    )


def test_a_failed_zstd_install_stops_before_the_download(tmp_path):
    result, log = _run(tmp_path, zstd_present=False, apt_status=1)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "could not install zstd" in result.stdout
    assert _logged(log, "curl -fsSL https://ollama.com/install.sh") == 0
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0


def test_a_failed_download_is_reported_and_does_not_pull(tmp_path):
    result, log = _run(tmp_path, curl_status=6)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert (
        "could not download https://ollama.com/install.sh (network unavailable?)"
        in result.stdout
    )
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0
    assert not any(line.startswith("ollama create ") for line in log)


def test_a_failed_installer_is_reported(tmp_path):
    result, log = _run(tmp_path, installer_status=1)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "official installer failed (rc=1)" in result.stdout
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0


def test_a_service_that_will_not_start_fails_with_a_readable_message(tmp_path):
    result, log = _run(tmp_path, ollama_installed=True, start_status=1)

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "could not start the ollama service" in result.stdout
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0
    assert _logged(log, "systemctl restart ollama") == 0


def test_a_missing_modelfile_fails_before_any_install(tmp_path):
    result, log = _run(
        tmp_path,
        backend_dir=tmp_path / "empty-backend",
        setup_dir=tmp_path / "empty-setup",
    )

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "Modelfile not found" in result.stdout
    assert log == []


def _skip_line(variant: str) -> str:
    return f"ollama: skipping variant {variant}; the local model requires 8 GiB"


@pytest.mark.parametrize("variant", INSTALLING_VARIANTS)
def test_generation_5_variants_install_the_local_model(tmp_path, variant):
    result, log = _run(tmp_path, hardware_variant=variant)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "rc1=0" in result.stdout
    assert "skipping variant" not in result.stdout
    assert "installing from https://ollama.com/install.sh" in result.stdout
    assert "2252 MiB available RAM" in result.stdout
    assert _logged(log, "curl -fsSL https://ollama.com/install.sh") == 1
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 1
    assert (
        _logged(log, f"ollama create qwen-fast -f {REPO_ROOT}/setup/ollama/Modelfile")
        == 1
    )


@pytest.mark.parametrize("variant", SKIPPING_VARIANTS)
def test_generation_4_variants_skip_without_installer_pull_or_create(tmp_path, variant):
    result, log = _run(tmp_path, hardware_variant=variant, runs=2)

    assert result.returncode == 0, result.stdout + result.stderr
    first, second = result.stdout.split("--- run 2 ---", 1)
    assert "rc1=0" in first
    assert _skip_line(variant) in first
    assert "rc2=0" in second
    assert _skip_line(variant) in second
    assert "installing from" not in result.stdout
    assert "pulling qwen2.5:1.5b" not in result.stdout
    assert "creating qwen-fast" not in result.stdout
    assert _logged(log, "curl -fsSL https://ollama.com/install.sh") == 0
    assert _logged(log, "sh") == 0
    assert _logged(log, "ollama pull qwen2.5:1.5b") == 0
    assert not any(line.startswith("ollama create ") for line in log)
    assert not any(line.startswith("systemctl ") for line in log)


def test_setup_runs_the_ollama_step_after_the_clone():
    text = SETUP_PIB.read_text(encoding="utf-8")

    clone = text.index('run_step "Clone repositories"')
    step = text.index('run_step "Install Ollama qwen-fast" install_ollama_qwen_fast')
    assert clone < step
    assert "curl -fsSL https://ollama.com/install.sh | sh" in text
    assert "systemctl restart ollama" not in text
    assert "function ensure_ollama_listen_dropin" not in text
    assert "installation_scripts/ollama_listen.sh" in text
    assert "required_mib=1200" in text
    assert "pib5edu | pib5advanced | pib5museum" in text


def test_the_listen_dropin_is_written_before_the_first_start(tmp_path):
    result, log = _run(tmp_path, runs=2)
    dropin = tmp_path / "ollama-override.conf"

    assert result.returncode == 0, result.stdout + result.stderr
    assert (
        dropin.read_text(encoding="utf-8")
        == '[Service]\nEnvironment="OLLAMA_HOST=0.0.0.0:11434"\n'
    )
    assert _logged(log, f"sudo /usr/bin/tee {dropin}") == 1
    assert log.index(f"sudo /usr/bin/tee {dropin}") < log.index(
        "systemctl start ollama"
    )
    assert _logged(log, "systemctl restart ollama") == 0
    assert (
        "container listen address already configured"
        in result.stdout.split("--- run 2 ---", 1)[1]
    )


def test_tags_that_omit_qwen_fast_fail_the_step(tmp_path):
    result, _log = _run(
        tmp_path,
        ollama_installed=True,
        models=["qwen2.5:1.5b", "qwen-fast:latest"],
        service_active=True,
        service_enabled=True,
        tags_body='{"models":[{"name":"qwen2.5:1.5b"}]}',
    )

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "/api/tags does not list qwen-fast" in result.stdout


def test_an_unreachable_tags_endpoint_fails_the_step(tmp_path):
    result, _log = _run(
        tmp_path,
        ollama_installed=True,
        models=["qwen-fast:latest"],
        service_active=True,
        service_enabled=True,
        tags_status=7,
    )

    assert "rc1=1" in result.stdout, result.stdout + result.stderr
    assert "did not answer" in result.stdout
