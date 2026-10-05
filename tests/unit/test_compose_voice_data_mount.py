"""The voice data directory must not point at a developer's machine.

docker-compose.yaml mounted a hard-coded host path for the voice data, so on a robot the container
mounted an empty directory and could not see the model weights at all. The path now comes from the
environment with /data/voice as the default.
"""

import re
from pathlib import Path

COMPOSE = Path(__file__).resolve().parents[2] / "docker-compose.yaml"


def test_no_developer_host_path_is_mounted():
    text = COMPOSE.read_text()
    assert (
        "/media/" not in text
    ), "a developer's home directory must not appear in the compose file"


def test_every_voice_data_mount_uses_the_environment_default():
    text = COMPOSE.read_text()
    mounts = re.findall(r"-\s*([^\s:]+:[^\s:]+:/data/voice)", text)
    assert (
        len(mounts) == 2
    ), f"expected the voice assistant and the audio io service, found {mounts}"
    for mount in mounts:
        assert mount.startswith("${PIB_VOICE_DATA_DIR:-/data/voice}"), mount
