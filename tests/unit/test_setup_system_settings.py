"""No step may do nothing and report success (PR-1910, findings 4 and 5).

The install log showed `cp: cannot create regular file ''` from
set_system_settings.sh: `local` at the top level of a sourced file is rejected by
bash, the variables stayed empty, and the step still printed SUCCESS. The file is
now functions only, each guarding its inputs with require_nonempty. The audio
step waited for nothing and hit "Translate ID error: '-1' is not a valid ID"; it
now waits for a default sink.

Everything runs for real in a bash subprocess against tmp_path with stubs on PATH.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
SYSTEM_SETTINGS = (
    REPO_ROOT / "setup" / "installation_scripts" / "set_system_settings.sh"
)

DESKTOP_FILE = """[Desktop Entry]
Name=Web Browser
Exec=x-www-browser %U
Type=Application
"""

# Executes file operations (the test points them at tmp_path), records the rest.
SUDO_STUB = """#!/bin/bash
printf '%s\\n' "$*" >> "$STUB_LOG"
case "$1" in
    sed|tee|cp) exec "$@" ;;
esac
exit 0
"""

PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""


def _extract(script: Path, name: str) -> str:
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) ?\{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _stubs(tmp_path: Path, extra: dict[str, str] | None = None) -> Path:
    stub_bin = tmp_path / "bin"
    stub_bin.mkdir(exist_ok=True)
    for name, body in {"sudo": SUDO_STUB, **(extra or {})}.items():
        (stub_bin / name).write_text(body, encoding="utf-8")
        (stub_bin / name).chmod(0o755)
    return stub_bin


def _run(tmp_path: Path, script: str, **env_overrides):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _stubs(tmp_path)
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        PIB_SYSTEM_SETTINGS_DEFINE_ONLY="1",
        SYSTEM_SETTINGS=str(SYSTEM_SETTINGS),
    )
    env.update({key: str(value) for key, value in env_overrides.items()})
    result = subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, env=env, text=True
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def _source_settings() -> str:
    """Prelude + require_nonempty from setup-pib.sh + the sourced settings file."""
    return (
        PRELUDE
        + _extract(SETUP_PIB, "require_nonempty")
        + '\nsource "$SYSTEM_SETTINGS"\n'
    )


# ---- finding 4: empty variables ------------------------------------------------------------


def test_the_sourced_file_declares_no_variables_outside_functions():
    text = SYSTEM_SETTINGS.read_text(encoding="utf-8")
    depth = 0
    for number, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if re.match(r"^(function )?\w+\(\) \{", stripped):
            depth += 1
        elif stripped == "}" and depth:
            depth -= 1
        elif stripped.startswith("local ") and depth == 0:
            raise AssertionError(f"line {number}: `local` outside a function: {line}")


def test_chromium_password_store_is_configured_in_the_users_desktop_file(tmp_path):
    home = tmp_path / "home"
    home.mkdir()
    source = tmp_path / "x-www-browser.desktop"
    source.write_text(DESKTOP_FILE, encoding="utf-8")

    result, _ = _run(
        tmp_path,
        _source_settings() + 'configure_chromium_password_store; echo "rc=$?"\n',
        HOME=home,
        PIB_BROWSER_DESKTOP_SOURCE=source,
    )

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    installed = home / ".local" / "share" / "applications" / "x-www-browser.desktop"
    assert "Exec=x-www-browser --password-store=basic %U" in installed.read_text()
    assert "Configured Chromium to use basic password store" in result.stdout


def test_an_empty_home_fails_loudly_instead_of_copying_to_nowhere(tmp_path):
    source = tmp_path / "x-www-browser.desktop"
    source.write_text(DESKTOP_FILE, encoding="utf-8")

    result, _ = _run(
        tmp_path,
        _source_settings() + 'configure_chromium_password_store; echo "rc=$?"\n',
        HOME="",
        PIB_BROWSER_DESKTOP_SOURCE=source,
    )

    assert "rc=1" in result.stdout, result.stdout + result.stderr
    assert "variable HOME is empty" in result.stdout
    assert "cannot create regular file ''" not in result.stderr
    assert "grep: :" not in result.stderr
    assert "sed: can't read :" not in result.stderr
    assert "Configured Chromium" not in result.stdout


def test_display_settings_are_written_to_the_boot_config(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text("[all]\ndtoverlay=vc4-kms-v3d\nhdmi_group=1\n", encoding="utf-8")

    result, _ = _run(
        tmp_path,
        _source_settings() + 'configure_display_settings; echo "rc=$?"\n',
        PIB_BOOT_CONFIG=config,
    )

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    text = config.read_text(encoding="utf-8")
    assert "dtoverlay=vc4-kms-v3d" in text.splitlines()
    assert "fkms" not in text
    assert "hdmi_group=2" in text and "hdmi_group=1" not in text
    assert "hdmi_cvt 1024 600 60 6 0 0 0" in text
    assert "Adjusted display resolution and settings" in result.stdout


# ---- PR-1980: full KMS on Raspberry Pi OS Wayland ------------------------------------------
#
# The step used to rewrite dtoverlay=vc4-kms-v3d to vc4-fkms-v3d on every run. On the
# Raspberry Pi 5 that left the kernel on the firmware KMS backend with "No displays found",
# labwc found 0 GPUs and LightDM gave up. The step now keeps full KMS and turns the line it
# wrote earlier back, with a backup, so a second setup run cannot undo the repair.

# Raspberry Pi OS (trixie) /boot/firmware/config.txt as shipped.
STOCK_CONFIG = """\
# For more options and information see
# http://rptl.io/configtxt
# Some settings may impact device functionality. See link above for details

# Uncomment some or all of these to enable the optional hardware interfaces
#dtparam=i2c_arm=on
#dtparam=i2s=on
#dtparam=spi=on

# Enable audio (loads snd_bcm2835)
dtparam=audio=on

# Automatically load overlays for detected cameras
camera_auto_detect=1

# Automatically load overlays for detected DSI displays
display_auto_detect=1

# Automatically load initramfs files, if found
auto_initramfs=1

# Enable DRM VC4 V3D driver
dtoverlay=vc4-kms-v3d
max_framebuffers=2

# Don't have the firmware create an initial video= setting in cmdline.txt.
# Use the kernel's default instead.
disable_fw_kms_setup=1

# Run in 64-bit mode
arm_64bit=1

# Disable compensation for displays with overscan
disable_overscan=1

# Run as fast as firmware / board allows
arm_boost=1

[cm4]
# Enable host mode on the 2711 built-in XHCI USB controller.
# This line should be removed if the legacy DWC2 controller is required
# (e.g. for USB device mode) or if USB support is not required.
otg_mode=1

[cm5]
dtoverlay=dwc2,dr_mode=host

[all]
"""

PANEL_DIRECTIVES = [
    "hdmi_force_edid_audio=1",
    "max_usb_current=1",
    "hdmi_force_hotplug=1",
    "config_hdmi_boost=7",
    "hdmi_group=2",
    "hdmi_mode=87",
    "hdmi_drive=2",
    "display_rotate=0",
    "hdmi_cvt 1024 600 60 6 0 0 0",
]

# What the earlier setup left on an installed robot (measured on the affected Pi 5).
AFFECTED_CONFIG = (
    STOCK_CONFIG.replace("dtoverlay=vc4-kms-v3d", "dtoverlay=vc4-fkms-v3d")
    + "\n".join(PANEL_DIRECTIVES)
    + "\n"
)


def _configure(tmp_path: Path, config: Path):
    return _run(
        tmp_path,
        _source_settings() + 'configure_display_settings; echo "rc=$?"\n',
        PIB_BOOT_CONFIG=config,
    )[0]


def _backups(config: Path) -> list[Path]:
    return sorted(config.parent.glob(config.name + ".pib-backup-*"))


def _without_panel_directives(text: str) -> list[str]:
    return [line for line in text.splitlines() if line not in PANEL_DIRECTIVES]


def test_a_fresh_install_keeps_full_kms_and_every_unrelated_line(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(STOCK_CONFIG, encoding="utf-8")

    result = _configure(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    text = config.read_text(encoding="utf-8")
    assert "fkms" not in text
    assert _without_panel_directives(text) == STOCK_CONFIG.splitlines()
    for directive in PANEL_DIRECTIVES:
        assert text.splitlines().count(directive) == 1, directive
    assert _backups(config) == []
    assert "reboot" not in result.stdout.lower()


def test_an_installed_fkms_line_is_restored_to_full_kms_with_a_backup(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(AFFECTED_CONFIG, encoding="utf-8")

    result = _configure(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    text = config.read_text(encoding="utf-8")
    assert "fkms" not in text
    lines = text.splitlines()
    # Same place as before: [all], directly above max_framebuffers.
    assert lines.index("dtoverlay=vc4-kms-v3d") == STOCK_CONFIG.splitlines().index(
        "dtoverlay=vc4-kms-v3d"
    )
    assert lines.count("dtoverlay=vc4-kms-v3d") == 1
    assert _without_panel_directives(text) == STOCK_CONFIG.splitlines()
    backups = _backups(config)
    assert len(backups) == 1
    assert backups[0].read_text(encoding="utf-8") == AFFECTED_CONFIG
    assert str(backups[0]) in result.stdout
    assert "reboot" in result.stdout.lower()


def test_repeated_setup_runs_change_nothing_after_the_repair(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(AFFECTED_CONFIG, encoding="utf-8")

    first = _configure(tmp_path, config)
    after_first = config.read_text(encoding="utf-8")
    second = _configure(tmp_path, config)
    third = _configure(tmp_path, config)

    assert "rc=0" in first.stdout and "rc=0" in second.stdout and "rc=0" in third.stdout
    assert config.read_text(encoding="utf-8") == after_first
    assert "fkms" not in after_first
    assert len(_backups(config)) == 1
    assert "reboot" not in second.stdout.lower()
    assert "reboot" not in third.stdout.lower()


def test_options_sections_and_comments_survive_the_repair(tmp_path):
    config = tmp_path / "config.txt"
    original = (
        "#dtoverlay=vc4-fkms-v3d\n"
        "[pi4]\n"
        "dtoverlay=vc4-fkms-v3d,cma-256\n"
        "[all]\n"
        "  dtoverlay=vc4-fkms-v3d\n"
        "dtparam=audio=on\n"
    )
    config.write_text(original, encoding="utf-8")

    result = _configure(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert _without_panel_directives(config.read_text(encoding="utf-8")) == [
        "#dtoverlay=vc4-fkms-v3d",
        "[pi4]",
        "dtoverlay=vc4-kms-v3d,cma-256",
        "[all]",
        "dtoverlay=vc4-kms-v3d",
        "dtparam=audio=on",
    ]
    assert len(_backups(config)) == 1


def test_a_board_specific_kms_overlay_counts_as_full_kms(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text("[all]\ndtoverlay=vc4-kms-v3d-pi5,cma-512\n", encoding="utf-8")

    result = _configure(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    lines = config.read_text(encoding="utf-8").splitlines()
    assert "dtoverlay=vc4-kms-v3d-pi5,cma-512" in lines
    assert "dtoverlay=vc4-kms-v3d" not in lines
    assert _backups(config) == []


def test_a_config_without_any_kms_overlay_gets_exactly_one(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(
        "# Enable DRM VC4 V3D driver\n[all]\narm_64bit=1", encoding="utf-8"
    )

    _configure(tmp_path, config)
    result = _configure(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    lines = config.read_text(encoding="utf-8").splitlines()
    assert lines[:3] == ["# Enable DRM VC4 V3D driver", "[all]", "arm_64bit=1"]
    assert lines.count("dtoverlay=vc4-kms-v3d") == 1
    assert "fkms" not in "\n".join(lines)


# ---- PR-1980: supported repair of an installed robot ----------------------------------------

REPAIR_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
get_distribution() { echo "${STUB_DISTRIBUTION:-debian}"; }
get_dist_version() { echo "${STUB_DIST_VERSION:-trixie}"; }
"""

ID_ROOT_AWARE_STUB = """#!/bin/bash
if [ "${1:-}" = "-u" ] && [ $# -eq 1 ]; then echo "${STUB_UID:-1000}"; exit 0; fi
exec /usr/bin/id "$@"
"""


def _repair(tmp_path: Path, config: Path, **env):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _stubs(tmp_path, {"id": ID_ROOT_AWARE_STUB})
    script = (
        REPAIR_PRELUDE
        + _extract(SETUP_PIB, "require_nonempty")
        + _extract(SETUP_PIB, "is_supported_raspbian")
        + _extract(SETUP_PIB, "repair_display_settings")
        + 'repair_display_settings; echo "rc=$?"\n'
    )
    environment = dict(os.environ)
    environment.update(
        PATH=f"{stub_bin}{os.pathsep}{environment['PATH']}",
        STUB_LOG=str(log),
        SETUP_SCRIPT_DIR=str(SETUP_PIB.parent),
        PIB_BOOT_CONFIG=str(config),
    )
    environment.update({key: str(value) for key, value in env.items()})
    return subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )


def test_the_repair_mode_restores_full_kms_and_asks_for_a_reboot(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(AFFECTED_CONFIG, encoding="utf-8")

    result = _repair(tmp_path, config)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert "fkms" not in config.read_text(encoding="utf-8")
    assert len(_backups(config)) == 1
    assert "reboot" in result.stdout.lower()
    # Only the display step ran: no Docker, no clone, no system settings step.
    assert "System settings adjusted" not in result.stdout
    assert "systemctl" not in (tmp_path / "stub.log").read_text(encoding="utf-8")


def test_the_repair_mode_refuses_root_and_unsupported_systems(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(AFFECTED_CONFIG, encoding="utf-8")

    as_root = _repair(tmp_path, config, STUB_UID=0)
    ubuntu = _repair(
        tmp_path, config, STUB_DISTRIBUTION="ubuntu", STUB_DIST_VERSION="noble"
    )

    assert "rc=1" in as_root.stdout, as_root.stdout + as_root.stderr
    assert "user pib" in as_root.stdout
    assert "rc=1" in ubuntu.stdout, ubuntu.stdout + ubuntu.stderr
    assert "Raspberry Pi OS" in ubuntu.stdout
    assert config.read_text(encoding="utf-8") == AFFECTED_CONFIG
    assert _backups(config) == []


def test_setup_dispatches_the_repair_mode_before_the_full_install():
    text = SETUP_PIB.read_text(encoding="utf-8")
    help_text = text.split("\nshow_help()\n", 1)[1].split("\n}\n", 1)[0]

    assert re.search(r"^\s+--repair-display\)\n\s+REPAIR_DISPLAY_ONLY=true", text, re.M)
    assert "--repair-display" in help_text
    dispatch = text.index('if [ "$REPAIR_DISPLAY_ONLY" = true ]; then')
    assert dispatch < text.index('LOG_FILE="$HOME/setup-pib.log"')
    assert dispatch < text.index("/etc/pib_smart_chats")
    assert dispatch < text.index('run_step "Install system packages"')
    assert "repair_display_settings\n  exit $?" in text[dispatch : dispatch + 200]


def test_setup_with_the_repair_flag_stops_before_anything_else_as_root(tmp_path):
    config = tmp_path / "config.txt"
    config.write_text(AFFECTED_CONFIG, encoding="utf-8")
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _stubs(tmp_path, {"id": ID_ROOT_AWARE_STUB})
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        STUB_UID="0",
        HOME=str(tmp_path),
        PIB_BOOT_CONFIG=str(config),
    )

    result = subprocess.run(
        ["bash", str(SETUP_PIB), "--repair-display"],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )

    assert result.returncode == 1, result.stdout + result.stderr
    assert "user pib" in result.stdout
    assert config.read_text(encoding="utf-8") == AFFECTED_CONFIG
    assert not (tmp_path / "setup-pib.log").exists()
    assert log.read_text(encoding="utf-8") == ""


def test_display_settings_name_a_missing_boot_config_and_an_empty_path_fails(tmp_path):
    missing, _ = _run(
        tmp_path,
        _source_settings() + 'configure_display_settings; echo "rc=$?"\n',
        PIB_BOOT_CONFIG=tmp_path / "absent.txt",
    )
    assert "rc=0" in missing.stdout
    assert "does not exist; display settings were not changed" in missing.stdout

    script = (
        _source_settings()
        + 'configure_display_settings() { local config_file=""; require_nonempty '
        'config_file || return 1; }\nconfigure_display_settings; echo "rc=$?"\n'
    )
    empty, _ = _run(tmp_path, script)
    assert "rc=1" in empty.stdout
    assert "variable config_file is empty" in empty.stdout


def test_set_system_settings_returns_non_zero_when_a_part_fails(tmp_path):
    script = _source_settings() + """
is_supported_raspbian() { return 0; }
is_ubuntu_noble() { return 1; }
DIST_VERSION=trixie
configure_display_settings() { return 0; }
configure_chromium_password_store() { echo "chromium part failed"; return 1; }
set_system_settings; echo "rc=$?"
"""
    result, _ = _run(tmp_path, script)

    assert "rc=1" in result.stdout, result.stdout + result.stderr
    assert "System settings were not fully adjusted" in result.stdout
    assert "[SUCCESS][[ System settings adjusted ]]" not in result.stdout


def test_require_nonempty_names_the_variable_and_the_caller():
    script = PRELUDE + _extract(SETUP_PIB, "require_nonempty") + """
caller_function() { local target=""; require_nonempty HOME target; }
caller_function; echo "rc=$?"
"""
    result = subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, text=True
    )

    assert "rc=1" in result.stdout
    assert "variable target is empty (caller_function)" in result.stdout


def test_setup_guards_the_desktop_copy_and_never_uses_set_u():
    text = SETUP_PIB.read_text(encoding="utf-8")
    move = _extract(SETUP_PIB, "move_setup_files")

    assert "require_nonempty HOME BACKEND_DIR || return 1" in move
    assert move.index("require_nonempty") < move.index("pib-eyes-animated.gif")
    assert not re.search(r"^\s*set -[a-zA-Z]*u", text, re.M)
    assert not re.search(r"^\s*set -o nounset", text, re.M)


# ---- finding 5: default audio sink ---------------------------------------------------------

ID_STUB = """#!/bin/bash
if [ "${1:-}" = "-u" ] && [ "${2:-}" = "pib" ]; then echo 1000; exit 0; fi
exec /usr/bin/id "$@"
"""

# `inspect @DEFAULT_AUDIO_SINK@` fails STUB_SINK_READY_AFTER times, as wpctl does while
# wireplumber has no default node yet; afterwards every call succeeds.
WPCTL_STUB = """#!/bin/bash
printf 'wpctl %s\\n' "$*" >> "$STUB_LOG"
if [ "$1" = "inspect" ]; then
    count=$(cat "$STUB_COUNTER" 2>/dev/null || echo 0)
    count=$((count + 1))
    echo "$count" > "$STUB_COUNTER"
    if [ "$count" -le "${STUB_SINK_READY_AFTER:-0}" ]; then
        echo "Translate ID error: '-1' is not a valid ID" >&2
        exit 1
    fi
    exit 0
fi
if [ "$1" = "get-volume" ]; then echo "Volume: 1.00"; fi
exit 0
"""

AUDIO_SUDO_STUB = """#!/bin/bash
args=()
skip=0
for arg in "$@"; do
    if [ "$skip" = 1 ]; then skip=0; continue; fi
    case "$arg" in
        -u) skip=1; continue ;;
        [A-Za-z_]*=*) continue ;;
    esac
    args+=("$arg")
done
exec "${args[@]}"
"""


def _run_volume(tmp_path: Path, ready_after: int, wait_seconds: int = 3):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _stubs(
        tmp_path, {"sudo": AUDIO_SUDO_STUB, "id": ID_STUB, "wpctl": WPCTL_STUB}
    )
    script = (
        PRELUDE
        + _extract(SETUP_PIB, "wait_for_default_audio_sink")
        + _extract(SETUP_PIB, "set_default_output_volume")
        + '\nset_default_output_volume; echo "rc=$?"\n'
    )
    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        STUB_COUNTER=str(tmp_path / "counter"),
        STUB_SINK_READY_AFTER=str(ready_after),
        PIB_WPCTL_WAIT_SECONDS=str(wait_seconds),
    )
    result = subprocess.run(
        ["bash", "-c", script], capture_output=True, check=False, env=env, text=True
    )
    return result, log.read_text(encoding="utf-8").splitlines()


def test_the_volume_is_set_only_after_a_default_sink_exists(tmp_path):
    result, calls = _run_volume(tmp_path, ready_after=2)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    inspects = [i for i, call in enumerate(calls) if call.startswith("wpctl inspect")]
    sets = [i for i, call in enumerate(calls) if "set-volume" in call]
    assert len(inspects) == 3, calls
    assert sets and sets[0] > inspects[-1], calls
    assert "Default output volume: Volume: 1.00" in result.stdout
    assert "Translate ID error" not in result.stdout


def test_without_a_default_sink_the_step_warns_and_touches_nothing(tmp_path):
    result, calls = _run_volume(tmp_path, ready_after=99, wait_seconds=2)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert "no default audio sink appeared within 2s" in result.stdout
    assert not [call for call in calls if "set-volume" in call], calls
