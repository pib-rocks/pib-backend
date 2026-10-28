"""PR-2086: duplex playback chooses Pulse or ALSA and logs the decision."""

from __future__ import annotations

import logging
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros_packages" / "voice_assistant"))


class _FakeStream:
    def write(self, data: bytes) -> None:
        pass

    def stop_stream(self) -> None:
        pass

    def close(self) -> None:
        pass


class _FakePyAudio:
    def __init__(
        self,
        *,
        host_apis: list[str],
        pulse_output: tuple[str, int] | None = ("pulse default", 2),
        alsa_output: tuple[str, int] = ("hdmi", 0),
        pulse_open_error: OSError | None = None,
    ) -> None:
        self.host_apis = host_apis
        self.pulse_output = pulse_output
        self.alsa_output = alsa_output
        self.pulse_open_error = pulse_open_error
        self.open_calls: list[dict] = []
        self.terminated = False

    def get_host_api_count(self) -> int:
        return len(self.host_apis)

    def get_host_api_info_by_index(self, index: int) -> dict:
        return {"index": index, "name": self.host_apis[index]}

    def get_device_count(self) -> int:
        return 3

    def get_default_output_device_info(self) -> dict:
        name, index = self.alsa_output
        return {
            "index": index,
            "name": name,
            "hostApi": 0,
            "maxOutputChannels": 2,
        }

    def get_device_info_by_index(self, index: int) -> dict:
        if self.pulse_output and index == self.pulse_output[1]:
            return {
                "index": index,
                "name": self.pulse_output[0],
                "hostApi": 1,
                "maxOutputChannels": 2,
            }
        name, idx = self.alsa_output
        return {
            "index": idx,
            "name": name,
            "hostApi": 0,
            "maxOutputChannels": 2,
        }

    def open(self, **kwargs):
        self.open_calls.append(dict(kwargs))
        if kwargs.get("output_device_index") == (
            self.pulse_output[1] if self.pulse_output else None
        ):
            if self.pulse_open_error is not None:
                raise self.pulse_open_error
        return _FakeStream()

    def terminate(self) -> None:
        self.terminated = True


def _install_pyaudio(fake: _FakePyAudio, monkeypatch: pytest.MonkeyPatch) -> None:
    module = types.ModuleType("pyaudio")
    module.paInt16 = 8
    module.PyAudio = lambda: fake
    monkeypatch.setitem(sys.modules, "pyaudio", module)


def test_open_speaker_uses_pulse_when_configured(caplog, monkeypatch) -> None:
    monkeypatch.setenv("PULSE_SERVER", "unix:/run/user/1000/pulse/native")
    fake = _FakePyAudio(host_apis=["ALSA", "PulseAudio"])
    _install_pyaudio(fake, monkeypatch)

    from voice_assistant import ros_duplex

    caplog.set_level(logging.INFO, logger="voice_assistant.ros_duplex")
    audio, stream = ros_duplex.open_speaker()

    assert stream is not None
    assert audio is fake
    assert len(fake.open_calls) == 1
    assert fake.open_calls[0]["output_device_index"] == 2
    assert any(
        "audio_output=pulse" in record.message
        and "device=pulse default" in record.message
        and "rate=16000" in record.message
        and "channels=1" in record.message
        for record in caplog.records
    )


def test_open_speaker_uses_alsa_without_pulse_server(caplog, monkeypatch) -> None:
    monkeypatch.delenv("PULSE_SERVER", raising=False)
    fake = _FakePyAudio(host_apis=["ALSA", "PulseAudio"])
    _install_pyaudio(fake, monkeypatch)

    from voice_assistant import ros_duplex

    caplog.set_level(logging.INFO, logger="voice_assistant.ros_duplex")
    audio, stream = ros_duplex.open_speaker()

    assert stream is not None
    assert audio is fake
    assert len(fake.open_calls) == 1
    assert "output_device_index" not in fake.open_calls[0]
    assert any("audio_output=alsa" in record.message for record in caplog.records)


def test_open_speaker_falls_back_to_alsa_when_pulse_open_fails(
    caplog, monkeypatch
) -> None:
    monkeypatch.setenv("PULSE_SERVER", "unix:/run/user/1000/pulse/native")
    fake = _FakePyAudio(
        host_apis=["ALSA", "PulseAudio"],
        pulse_open_error=OSError(-9999, "no pulse sink"),
    )
    _install_pyaudio(fake, monkeypatch)

    from voice_assistant import ros_duplex

    caplog.set_level(logging.DEBUG, logger="voice_assistant.ros_duplex")
    audio, stream = ros_duplex.open_speaker()

    assert stream is not None
    assert audio is fake
    assert len(fake.open_calls) == 2
    assert fake.open_calls[0]["output_device_index"] == 2
    assert "output_device_index" not in fake.open_calls[1]
    assert any(
        "pulse playback open failed" in record.message
        and "no pulse sink" in record.message
        for record in caplog.records
    )
    assert any("audio_output=alsa" in record.message for record in caplog.records)


def test_open_speaker_missing_sink_does_not_raise(caplog, monkeypatch) -> None:
    monkeypatch.setenv("PULSE_SERVER", "unix:/run/user/1000/pulse/native")

    class _AlwaysFailPyAudio(_FakePyAudio):
        def open(self, **kwargs):
            self.open_calls.append(dict(kwargs))
            raise OSError(-9999, "no pulse sink")

    fake = _AlwaysFailPyAudio(host_apis=["ALSA", "PulseAudio"])
    _install_pyaudio(fake, monkeypatch)

    from voice_assistant import ros_duplex

    caplog.set_level(logging.ERROR, logger="voice_assistant.ros_duplex")
    audio, stream = ros_duplex.open_speaker()

    assert stream is None
    assert audio is None
    assert fake.terminated
    assert any(
        "audio playback unavailable" in record.message
        and "no pulse sink" in record.message
        for record in caplog.records
    )
