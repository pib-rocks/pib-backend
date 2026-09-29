"""Tests for the wireplumber volume drop-in shipped and installed by setup-pib.sh.

A sink route wireplumber has never seen falls back to its own default sink volume
(0.064 in the stock config, displayed as 40 %), so a speaker moved to another USB
port comes up quiet even though setup set the volume of the sink it saw back then.
The drop-in pins that fallback to 1.0.

install_wireplumber_volume_defaults() is executed for real: its source is cut out of
setup-pib.sh and run in a bash subprocess with stubs on PATH. `sudo` only executes
the file operations, which go into tmp_path; everything else is recorded.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
DROP_IN = REPO_ROOT / "setup" / "setup_files" / "50-pib-volume.conf"

PIB_UID = "1000"
RESTART_CALL = (
    f"XDG_RUNTIME_DIR=/run/user/{PIB_UID} systemctl --user restart wireplumber"
)

# Records every sudo invocation and runs only the file operations, which the test
# points at tmp_path. The recorded line keeps the leading VAR=value assignments so
# the tests can assert wireplumber is addressed through pib's runtime directory.
SUDO_STUB = """#!/bin/bash
args=()
skip_next=0
for arg in "$@"; do
    if [ "$skip_next" = "1" ]; then
        skip_next=0
        continue
    fi
    if [ "$arg" = "-u" ]; then
        skip_next=1
        continue
    fi
    args+=("$arg")
done
printf '%s\\n' "${args[*]}" >> "$STUB_LOG"

for arg in "${args[@]}"; do
    case "$arg" in
        [A-Za-z_]*=*) continue ;;
        mkdir|cp) exec "${args[@]}" ;;
        systemctl) exit "${STUB_RESTART_STATUS:-0}" ;;
        *) exit 0 ;;
    esac
done
exit 0
"""

# `id -u pib` decides whether the function runs at all; keep it independent of the host.
ID_STUB = f"""#!/bin/bash
if [ "${{1:-}}" = "-u" ] && [ "${{2:-}}" = "pib" ]; then
    if [ "${{STUB_PIB_EXISTS:-1}}" = "1" ]; then
        echo "{PIB_UID}"
        exit 0
    fi
    echo "id: 'pib': no such user" >&2
    exit 1
fi
exec /usr/bin/id "$@"
"""

WPCTL_STUB = """#!/bin/bash
printf 'wpctl %s\\n' "$*" >> "$STUB_LOG"
exit 0
"""

# print() and command_exists() come from the top of setup-pib.sh; sourcing the script
# itself would run the installer.
SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
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
        ("id", ID_STUB),
        ("wpctl", WPCTL_STUB),
    ):
        stub = stub_bin / name
        stub.write_text(body, encoding="utf-8")
        stub.chmod(0o755)
    return stub_bin


def _run_install(tmp_path: Path, backend_dir: Path | None = None, **env_overrides):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _make_stubs(tmp_path)
    conf_dir = tmp_path / "wireplumber.conf.d"

    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "install_wireplumber_volume_defaults")
        + "\ninstall_wireplumber_volume_defaults\necho rc=$?\n"
    )

    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        BACKEND_DIR=str(backend_dir if backend_dir is not None else REPO_ROOT),
        PIB_WIREPLUMBER_CONF_DIR=str(conf_dir),
    )
    env.update({key: str(value) for key, value in env_overrides.items()})

    result = subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )
    return result, log.read_text(encoding="utf-8").splitlines(), conf_dir


def _settings(text: str) -> dict[str, str]:
    return {
        key: value
        for key, value in re.findall(r"^\s*([\w.-]+)\s*=\s*([\d.]+)\s*$", text, re.M)
    }


def test_the_shipped_drop_in_pins_both_volume_defaults_to_full():
    text = DROP_IN.read_text(encoding="utf-8")

    assert "wireplumber.settings" in text
    assert _settings(text) == {
        "device.routes.default-sink-volume": "1.0",
        "node.stream.default-playback-volume": "1.0",
    }


def test_the_drop_in_is_installed_into_the_users_wireplumber_config_dir(tmp_path):
    result, _, conf_dir = _run_install(tmp_path)

    installed = conf_dir / "50-pib-volume.conf"
    assert installed.is_file(), result.stdout + result.stderr
    assert installed.read_text(encoding="utf-8") == DROP_IN.read_text(encoding="utf-8")
    assert "rc=0" in result.stdout


def test_the_installed_file_belongs_to_pib(tmp_path):
    _, calls, conf_dir = _run_install(tmp_path)

    assert f"chown -R pib:pib {conf_dir}" in calls


def test_wireplumber_is_restarted_for_pib_so_no_reboot_is_needed(tmp_path):
    result, calls, _ = _run_install(tmp_path)

    assert RESTART_CALL in calls
    assert "Restarted wireplumber" in result.stdout


def test_a_failing_restart_is_reported_without_failing_the_setup(tmp_path):
    result, calls, conf_dir = _run_install(tmp_path, STUB_RESTART_STATUS=1)

    assert (conf_dir / "50-pib-volume.conf").is_file()
    assert RESTART_CALL in calls
    assert "could not restart wireplumber" in result.stdout
    assert "Restarted wireplumber" not in result.stdout
    assert "rc=0" in result.stdout


def test_nothing_is_installed_when_the_user_pib_does_not_exist(tmp_path):
    result, calls, conf_dir = _run_install(tmp_path, STUB_PIB_EXISTS=0)

    assert not conf_dir.exists()
    assert not [call for call in calls if "systemctl" in call], calls
    assert "user 'pib' does not exist" in result.stdout
    assert "rc=0" in result.stdout


def test_a_missing_drop_in_is_an_error_and_wireplumber_is_left_alone(tmp_path):
    result, calls, _ = _run_install(tmp_path, backend_dir=tmp_path / "no-backend")

    assert "wireplumber drop-in not found" in result.stdout
    assert not [call for call in calls if "systemctl" in call], calls
    assert "rc=1" in result.stdout


def test_setup_installs_the_drop_in_and_keeps_the_existing_volume_step():
    text = SETUP_PIB.read_text(encoding="utf-8")

    install = text.index("\ninstall_wireplumber_volume_defaults ||")
    set_volume = text.index("\nset_default_output_volume ||")
    assert install < set_volume, "the drop-in must be in place before the sink is set"
