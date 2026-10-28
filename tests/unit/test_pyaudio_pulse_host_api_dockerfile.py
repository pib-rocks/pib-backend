"""PR-2085: voice and audio-io images build PyAudio with the PulseAudio host API.

Measured on the test robot: without libpulse-dev at wheel-build time, PortAudio
only registers ALSA and OSS, the ReSpeaker PCM node stays busy under PipeWire,
and capture never opens. ALSA remains available when PULSE_SERVER is unset.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]

AUDIO_DOCKERFILES = (
    "ros_packages/voice_assistant/Dockerfile",
    "ros_packages/ros_audio_io/Dockerfile",
)

# Both images run this guard after pip installs pyaudio (ephemeral user daemon for init).
PYAUDIO_PULSE_HOST_API_GUARD = "assert any('pulse' in n.lower() for n in apis)"
PORTAUDIO_PULSE_CMAKE_FLAG = "PA_USE_PULSEAUDIO=ON"
PORTAUDIO_OPTIONAL_PULSE_PATCH = "hostApis_[hostApisCount_] = NULL"
PORTAUDIO_PINNED_REVISION = "147dd722548358763a8b649b3e4b41dfffbcfbb6"


@pytest.mark.parametrize("relative", AUDIO_DOCKERFILES)
def test_libpulse_dev_is_installed_before_pyaudio(relative: str) -> None:
    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    assert "libpulse-dev" in text, relative
    assert PORTAUDIO_PULSE_CMAKE_FLAG in text, relative
    assert PORTAUDIO_OPTIONAL_PULSE_PATCH in text, relative
    assert text.index("libpulse-dev") < text.index("pyaudio=="), relative
    assert text.index(PORTAUDIO_PULSE_CMAKE_FLAG) < text.index("pyaudio=="), relative


@pytest.mark.parametrize("relative", AUDIO_DOCKERFILES)
def test_portaudio_source_is_pinned_to_a_revision(relative: str) -> None:
    """A moving master would change the patched file under us."""

    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    assert PORTAUDIO_PINNED_REVISION in text, relative
    assert (
        "git clone --depth 1 https://github.com/PortAudio/portaudio.git" not in text
    ), relative


@pytest.mark.parametrize("relative", AUDIO_DOCKERFILES)
def test_portaudio_and_pulse_client_stay_for_alsa_fallback(relative: str) -> None:
    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    assert "portaudio19-dev" in text, relative
    assert re.search(r"\bpulseaudio\b", text), relative


@pytest.mark.parametrize("relative", AUDIO_DOCKERFILES)
def test_image_build_asserts_pyaudio_exposes_pulse_host_api(relative: str) -> None:
    text = (REPO_ROOT / relative).read_text(encoding="utf-8")
    assert "import pyaudio" in text, relative
    assert "get_host_api_count" in text, relative
    assert PYAUDIO_PULSE_HOST_API_GUARD in text, relative
    assert "pulseaudio --daemonize" in text, relative
    assert text.index("pyaudio==") < text.index(PYAUDIO_PULSE_HOST_API_GUARD), relative
