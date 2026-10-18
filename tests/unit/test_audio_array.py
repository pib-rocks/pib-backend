"""PC-88: array detection, channel extraction, HID tuning and the AEC measurement."""

from __future__ import annotations

import numpy as np
import pytest

from voice_assistant.audio_devices import choose_input, extract_channel
from voice_assistant.hid_tuning import (
    DEFAULT_TUNING,
    HidDevice,
    apply_tuning,
    erle_db,
)


class FakeUsbDevice:
    def __init__(self, fail: set[str] | None = None) -> None:
        self.writes: list[tuple[int, bytes]] = []
        self.fail = fail or set()

    def ctrl_transfer(self, request_type, request, value, index, data, length=None):
        payload = bytes(data)
        name = payload[1:17].split(b"\x00", 1)[0].decode("ascii")
        if name in self.fail:
            raise OSError("control transfer refused")
        self.writes.append((request_type, payload))
        return bytearray(64)


# ----------------------------------------------------------------- device choice


def test_the_array_is_found_by_any_of_its_names() -> None:
    choice = choose_input(["Built-in Audio", "ReSpeaker 4 Mic Array (UAC1.0)"])
    assert choice.is_array is True
    assert choice.channel_index == 0
    assert "array" in choice.reason


def test_without_the_array_the_default_input_is_used_and_says_so() -> None:
    choice = choose_input(["Built-in Audio", "USB Headset"], default="Built-in Audio")
    assert choice.is_array is False
    assert choice.name == "Built-in Audio"
    assert "no microphone array" in choice.reason


def test_no_devices_at_all_does_not_raise() -> None:
    choice = choose_input([])
    assert choice.name == ""
    assert choice.is_array is False


# ----------------------------------------------------------------- channel


def test_six_channel_capture_yields_channel_zero_only() -> None:
    frames = np.arange(6 * 10, dtype=np.int16).reshape(10, 6)
    mono = extract_channel(frames, 6)
    assert mono.tolist() == frames[:, 0].tolist()
    assert mono.dtype == np.int16


def test_a_single_channel_stream_is_passed_through() -> None:
    frames = np.array([1, 2, 3, 4], dtype=np.int16)
    assert extract_channel(frames, 1).tolist() == [1, 2, 3, 4]


def test_a_flat_interleaved_buffer_is_reshaped() -> None:
    flat = np.array([0, 1, 2, 3, 4, 5], dtype=np.int16)  # two 3-channel frames
    assert extract_channel(flat, 3).tolist() == [0, 3]


@pytest.mark.parametrize(
    ("frames", "channels", "channel"),
    [
        (np.zeros(7, dtype=np.int16), 3, 0),  # not a multiple
        (np.zeros((4, 6), dtype=np.int16), 4, 0),  # width mismatch
        (np.zeros(8, dtype=np.int16), 2, 5),  # channel out of range
        (np.zeros(8, dtype=np.int16), 0, 0),  # nonsense
    ],
)
def test_impossible_channel_requests_are_refused(frames, channels, channel) -> None:
    with pytest.raises(ValueError):
        extract_channel(frames, channels, channel)


# ----------------------------------------------------------------- HID


def test_tuning_writes_the_three_switches() -> None:
    device = FakeUsbDevice()
    outcome = apply_tuning(HidDevice(device))
    assert outcome.ok is True
    assert set(outcome.applied) == {name for name, _ in DEFAULT_TUNING}
    names = [
        payload[1:17].split(b"\x00", 1)[0].decode() for _, payload in device.writes
    ]
    assert names == ["ECHOONOFF", "NLATTENONOFF", "AGCONOFF"]
    values = [payload[17] for _, payload in device.writes]
    assert values == [1, 1, 0]


def test_a_refused_parameter_is_reported_and_the_rest_still_applied() -> None:
    device = FakeUsbDevice(fail={"NLATTENONOFF"})
    outcome = apply_tuning(HidDevice(device))
    assert outcome.ok is False
    assert [name for name, _ in outcome.refused] == ["NLATTENONOFF"]
    assert "ECHOONOFF" in outcome.applied


def test_without_a_device_every_parameter_is_refused_but_nothing_raises() -> None:
    outcome = apply_tuning(None)
    assert outcome.ok is False
    assert len(outcome.refused) == len(DEFAULT_TUNING)
    assert outcome.refused[0][1] == "no HID device"


# ----------------------------------------------------------------- AEC measurement


def test_identical_signals_measure_zero_db() -> None:
    signal = np.sin(np.linspace(0, 40, 4000)) * 3000
    assert erle_db(signal, signal) == pytest.approx(0.0, abs=0.01)


def test_a_quiet_capture_measures_the_reduction() -> None:
    signal = np.sin(np.linspace(0, 40, 4000)) * 3000
    assert erle_db(signal, signal * 0.1) == pytest.approx(20.0, abs=0.1)


def test_the_measurement_refuses_nonsense_inputs() -> None:
    with pytest.raises(ValueError):
        erle_db(np.zeros(0), np.zeros(10))
    with pytest.raises(ValueError):
        erle_db(np.zeros(10), np.zeros(0))
    with pytest.raises(ValueError):
        erle_db(np.zeros(10), np.zeros(10))  # silent reference
