"""Tests for the power/watchdog step of setup-pib.sh and the venv guard of run_all_tests.sh.

The shell functions are executed for real: their source is cut out of the shipped
scripts and run in a bash subprocess with stub executables on PATH. `sudo` only
writes when the test asks for it (STUB_SUDO_EXEC=1), so nothing outside tmp_path
is ever touched - `/boot/firmware/config.txt` in particular is replaced by
PIB_BOOT_CONFIG.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
RUN_ALL_TESTS = REPO_ROOT / "tests" / "run_all_tests.sh"

# A shipped Raspberry Pi OS config.txt, shortened to the parts that matter here.
CONFIG_TXT = """\
# For more options and information see
# http://rptl.io/configtxt

[all]
dtparam=audio=on
camera_auto_detect=1
display_auto_detect=1
auto_initramfs=1
dtoverlay=vc4-kms-v3d
max_framebuffers=2
disable_fw_kms_setup=1
arm_64bit=1
disable_overscan=1
arm_boost=1

[cm5]
dtoverlay=dwc2,dr_mode=host
"""

# Records every sudo invocation (leading VAR=value assignments dropped) and only
# runs the command when the test explicitly enables it.
SUDO_STUB = """#!/bin/bash
args=()
for arg in "$@"; do
    if [ ${#args[@]} -eq 0 ] && [[ "$arg" == [A-Za-z_]*=* ]]; then
        continue
    fi
    args+=("$arg")
done
printf '%s\\n' "${args[*]}" >> "$STUB_LOG"
if [ "${STUB_SUDO_EXEC:-0}" = "1" ]; then
    exec "${args[@]}"
fi
exit 0
"""

# `list-unit-files` reports the watchdog unit only when the test says it is installed,
# which is how the real systemctl behaves with and without the `watchdog` package.
SYSTEMCTL_STUB = """#!/bin/bash
printf 'systemctl %s\\n' "$*" >> "$STUB_LOG"
if [ "${1:-}" = "list-unit-files" ]; then
    if [ "${STUB_WATCHDOG_UNIT:-0}" = "1" ]; then
        echo "UNIT FILE        STATE   PRESET"
        echo "watchdog.service enabled enabled"
        echo ""
        echo "1 unit files listed."
        exit 0
    fi
    echo "0 unit files listed."
    exit 1
fi
exit 0
"""

APT_GET_STUB = """#!/bin/bash
printf 'apt-get %s\\n' "$*" >> "$STUB_LOG"
exit 0
"""

# `python3 -m venv DIR` builds the minimum this script needs from a venv.
PYTHON3_STUB = """#!/bin/bash
printf 'python3 %s\\n' "$*" >> "$STUB_LOG"
if [ "${1:-}" = "-m" ] && [ "${2:-}" = "venv" ]; then
    target="$3"
    mkdir -p "$target/bin"
    printf '#!/bin/bash\\nexit 0\\n' > "$target/bin/python"
    chmod 755 "$target/bin/python"
    : > "$target/bin/activate"
fi
exit 0
"""

PIP_STUB = """#!/bin/bash
printf 'pip %s\\n' "$*" >> "$STUB_LOG"
exit 0
"""

PLAYWRIGHT_STUB = """#!/bin/bash
printf 'playwright %s\\n' "$*" >> "$STUB_LOG"
exit 0
"""

# print() and command_exists() come from the top of setup-pib.sh; sourcing the script
# itself would run the installer.
SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""

# The globals run_all_tests.sh sets before main() reaches ensure_venv.
RUN_ALL_PRELUDE = """
RED=''
GREEN=''
YELLOW=''
NC=''
SKIP_ROBOT=1
INCLUDE_FRONTEND=0
section() { echo "========== $1 =========="; }
"""


def _extract_bash_function(script: Path, name: str) -> str:
    """Return the verbatim definition of a shell function from a script."""
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) \{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _make_stubs(tmp_path: Path) -> Path:
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir()
    for name, body in (
        ("sudo", SUDO_STUB),
        ("systemctl", SYSTEMCTL_STUB),
        ("apt-get", APT_GET_STUB),
        ("python3", PYTHON3_STUB),
        ("pip", PIP_STUB),
        ("playwright", PLAYWRIGHT_STUB),
    ):
        stub = stub_bin / name
        stub.write_text(body, encoding="utf-8")
        stub.chmod(0o755)
    return stub_bin


def _run_bash(tmp_path: Path, script: str, **env_overrides):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _make_stubs(tmp_path)

    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        # Never let a stray write reach the real boot config.
        PIB_BOOT_CONFIG=str(tmp_path / "unused-config.txt"),
    )
    env.update({key: str(value) for key, value in env_overrides.items()})

    result = subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def _setup_script(*function_names: str, body: str) -> str:
    functions = "\n".join(
        _extract_bash_function(SETUP_PIB, name) for name in function_names
    )
    return SETUP_PRELUDE + functions + body


def _run_disable_power_notification(tmp_path: Path, config: Path, runs: int = 1, **env):
    body = "\n".join("disable_power_notification\necho rc=$?" for _ in range(runs))
    script = _setup_script(
        "append_config_directive_once",
        "disable_watchdog_daemon",
        "disable_power_notification",
        body="\n" + body + "\n",
    )
    return _run_bash(
        tmp_path,
        script,
        PIB_BOOT_CONFIG=str(config),
        **env,
    )


def test_two_runs_leave_exactly_one_avoid_warnings_and_one_force_turbo(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    result, _ = _run_disable_power_notification(
        tmp_path, config, runs=2, STUB_SUDO_EXEC=1
    )

    assert result.returncode == 0, result.stdout + result.stderr
    lines = config.read_text(encoding="utf-8").splitlines()
    assert lines.count("avoid_warnings=2") == 1, lines
    assert lines.count("force_turbo=1") == 1, lines
    # The rest of the file is untouched: the original content is still there, in order.
    assert lines[: len(CONFIG_TXT.splitlines())] == CONFIG_TXT.splitlines()
    assert "is already set in" in result.stdout


def test_first_run_on_a_fresh_image_adds_both_directives(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    result, _ = _run_disable_power_notification(tmp_path, config, STUB_SUDO_EXEC=1)

    assert result.returncode == 0, result.stdout + result.stderr
    lines = config.read_text(encoding="utf-8").splitlines()
    assert lines.count("avoid_warnings=2") == 1
    assert lines.count("force_turbo=1") == 1


def test_directive_is_not_glued_onto_an_unterminated_last_line(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT.rstrip("\n"), encoding="utf-8")

    _run_disable_power_notification(tmp_path, config, runs=2, STUB_SUDO_EXEC=1)

    lines = config.read_text(encoding="utf-8").splitlines()
    assert lines.count("avoid_warnings=2") == 1, lines
    assert lines.count("force_turbo=1") == 1, lines
    assert lines[-3] == "dtoverlay=dwc2,dr_mode=host"


def test_a_commented_out_directive_is_not_mistaken_for_an_active_one(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT + "#force_turbo=1\n", encoding="utf-8")

    _run_disable_power_notification(tmp_path, config, STUB_SUDO_EXEC=1)

    lines = config.read_text(encoding="utf-8").splitlines()
    assert lines.count("force_turbo=1") == 1
    assert "#force_turbo=1" in lines


def test_no_second_watchdog_owner_is_installed_enabled_or_started(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    _, calls = _run_disable_power_notification(tmp_path, config, STUB_WATCHDOG_UNIT=0)

    assert not [call for call in calls if "apt-get install" in call], calls
    assert not [call for call in calls if "systemctl enable" in call], calls
    assert not [call for call in calls if "systemctl start" in call], calls


def test_the_two_no_op_steps_are_gone(tmp_path):
    """No sed against /etc/watchdog.conf and no kernel.panic knob that never worked."""
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    result, calls = _run_disable_power_notification(tmp_path, config)

    assert not [call for call in calls if "/etc/watchdog.conf" in call], calls
    assert not [call for call in calls if "kernel.panic" in call], calls
    assert not [call for call in calls if "sysctl" in call], calls
    assert "watchdog configuration" not in result.stdout
    assert "kernel panic" not in result.stdout


def test_an_already_installed_watchdog_daemon_is_disabled(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    result, calls = _run_disable_power_notification(
        tmp_path, config, STUB_WATCHDOG_UNIT=1
    )

    assert "systemctl list-unit-files watchdog.service" in calls
    assert "systemctl disable --now watchdog" in calls
    assert "rc=0" in result.stdout


def test_nothing_is_disabled_when_the_watchdog_daemon_is_absent(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(CONFIG_TXT, encoding="utf-8")

    result, calls = _run_disable_power_notification(
        tmp_path, config, STUB_WATCHDOG_UNIT=0
    )

    assert "systemctl list-unit-files watchdog.service" in calls
    assert not [call for call in calls if "systemctl disable" in call], calls
    assert "Watchdog daemon is not installed" in result.stdout
    assert "rc=0" in result.stdout


def _run_ensure_venv(tmp_path: Path, venv_dir: Path):
    script = (
        RUN_ALL_PRELUDE
        + f'VENV_DIR="{venv_dir}"\n'
        + f'SCRIPT_DIR="{tmp_path}"\n'
        + f'REPO_ROOT="{tmp_path}"\n'
        + _extract_bash_function(RUN_ALL_TESTS, "ensure_venv")
        + "\nensure_venv\necho rc=$?\n"
    )
    return _run_bash(tmp_path, script)


def test_a_venv_directory_without_bin_is_rebuilt_and_the_run_continues(tmp_path):
    """The failure this fixes: an interrupted `python3 -m venv` left a bin-less directory."""
    venv_dir = tmp_path / ".test-venv"
    (venv_dir / "lib").mkdir(parents=True)

    result, calls = _run_ensure_venv(tmp_path, venv_dir)

    assert "No such file or directory" not in result.stderr
    assert "Rebuilding unusable" in result.stdout
    assert f"python3 -m venv {venv_dir}" in calls
    assert (venv_dir / "bin" / "activate").is_file()
    assert "venv ready" in result.stdout
    assert "rc=0" in result.stdout


def test_a_missing_venv_is_created(tmp_path):
    venv_dir = tmp_path / ".test-venv"

    result, calls = _run_ensure_venv(tmp_path, venv_dir)

    assert f"Creating {venv_dir}" in result.stdout
    assert f"python3 -m venv {venv_dir}" in calls
    assert "venv ready" in result.stdout


def test_a_usable_venv_is_kept(tmp_path):
    venv_dir = tmp_path / ".test-venv"
    (venv_dir / "bin").mkdir(parents=True)
    python = venv_dir / "bin" / "python"
    python.write_text("#!/bin/bash\nexit 0\n", encoding="utf-8")
    python.chmod(0o755)
    (venv_dir / "bin" / "activate").write_text("", encoding="utf-8")
    marker = venv_dir / "marker"
    marker.write_text("keep me\n", encoding="utf-8")

    result, calls = _run_ensure_venv(tmp_path, venv_dir)

    assert not [call for call in calls if "-m venv" in call], calls
    assert marker.is_file()
    assert "venv ready" in result.stdout
