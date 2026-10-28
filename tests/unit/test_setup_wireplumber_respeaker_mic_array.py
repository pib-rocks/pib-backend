"""Tests for the WirePlumber ReSpeaker mic-array drop-in from setup-pib.sh.

PipeWire must expose the Seeed ReSpeaker as a six-channel capture source whose
channel 0 is the XVF-3000 processed stream. A silent stereo downmix would mix raw
microphones with channel 0 and the voice stack would hear pib's own playback.

install_wireplumber_respeaker_mic_array() is exercised like the volume drop-in:
its source is cut from setup-pib.sh and run under bash with stubs on PATH.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from test_setup_wireplumber_volume import (
    PIB_UID,
    REPO_ROOT,
    RESTART_CALL,
    SETUP_PIB,
    SETUP_PRELUDE,
    _extract_bash_function,
    _make_stubs,
)

DROP_IN = REPO_ROOT / "setup" / "setup_files" / "51-pib-respeaker-mic-array.conf"
INSTALL_FN = "install_wireplumber_respeaker_mic_array"


def _run_install(tmp_path: Path, backend_dir: Path | None = None, **env_overrides):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _make_stubs(tmp_path)
    conf_dir = tmp_path / "wireplumber.conf.d"

    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, INSTALL_FN)
        + f"\n{INSTALL_FN}\necho rc=$?\n"
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


def _run_install_twice(tmp_path: Path, **env_overrides):
    log = tmp_path / "stub.log"
    log.touch()
    stub_bin = _make_stubs(tmp_path)
    conf_dir = tmp_path / "wireplumber.conf.d"

    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, INSTALL_FN)
        + f"\n{INSTALL_FN}\n{INSTALL_FN}\necho rc=$?\n"
    )

    env = dict(os.environ)
    env.update(
        PATH=f"{stub_bin}{os.pathsep}{env['PATH']}",
        STUB_LOG=str(log),
        BACKEND_DIR=str(REPO_ROOT),
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
    return result, conf_dir


def test_the_shipped_drop_in_keeps_respeaker_capture_at_six_channels_without_downmix():
    text = DROP_IN.read_text(encoding="utf-8")

    assert "monitor.alsa.rules" in text
    assert "audio.channels = 6" in text
    assert "audio.rate = 16000" in text
    assert "channelmix.disable = true" in text
    assert "priority.session = 2500" in text
    assert "respeaker|seeed|uac1" in text
    assert "audio_devices.py" in text


def test_the_drop_in_is_installed_into_the_users_wireplumber_config_dir(tmp_path):
    result, _, conf_dir = _run_install(tmp_path)

    installed = conf_dir / "51-pib-respeaker-mic-array.conf"
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


def test_installing_twice_is_idempotent(tmp_path):
    result, conf_dir = _run_install_twice(tmp_path)

    installed = conf_dir / "51-pib-respeaker-mic-array.conf"
    assert installed.is_file()
    assert installed.read_text(encoding="utf-8") == DROP_IN.read_text(encoding="utf-8")
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


def test_setup_installs_the_drop_in_after_the_volume_drop_in():
    text = SETUP_PIB.read_text(encoding="utf-8")

    volume = text.index(
        'run_step "Install wireplumber volume drop-in" install_wireplumber_volume_defaults ||'
    )
    array_drop_in = text.index(
        'run_step "Install wireplumber ReSpeaker mic array drop-in" '
        "install_wireplumber_respeaker_mic_array ||"
    )
    set_volume = text.index(
        'run_step "Set default output volume" set_default_output_volume ||'
    )
    assert volume < array_drop_in < set_volume
