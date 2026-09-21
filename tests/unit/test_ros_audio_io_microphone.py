"""Plain-Python tests for ros-audio-io microphone logic (no ROS required)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROS_AUDIO_PACKAGE = Path(__file__).parents[2] / "ros_packages" / "ros_audio_io"
sys.path.insert(0, str(ROS_AUDIO_PACKAGE))

from ros_audio_io.levels import calculate_levels  # noqa: E402
from ros_audio_io.microphone_parameters import (  # noqa: E402
    DEFAULT_READBACK_TOLERANCE,
    PRESETS,
    TUNABLE_PARAMETERS,
    apply_tuning_values,
    readback_matches,
    readback_mismatch_reason,
    readback_tolerance,
    validate_parameter,
)

# Read-backs measured on 192.168.1.172 with a ReSpeaker Mic Array v2.0.
MEASURED_AGCTIME_READBACKS = {
    0.5: 0.9685218567028642,
    1.0: 0.9841422392055392,
}


class QuantizingDevice:
    """Device double that answers writes the way the XVF3000 was measured to."""

    def __init__(self):
        self.written = {}

    def write(self, name, value):
        self.written[name] = value

    def read(self, name):
        value = self.written[name]
        if name == "AGCTIME":
            # A KeyError here means a preset asks for an AGCTIME whose
            # read-back nobody has measured on the device.
            return MEASURED_AGCTIME_READBACKS[value]
        return value


class BrokenDevice(QuantizingDevice):
    """Device double whose second write fails, as a USB timeout would."""

    def write(self, name, value):
        if self.written:
            raise OSError("usb timeout")
        super().write(name, value)


def test_levels_silence_floor():
    assert calculate_levels([0] * 1024) == (0.0, 0.0)


def test_levels_full_scale():
    assert calculate_levels([-32768, 32767]) == pytest.approx(
        (math.sqrt((32768**2 + 32767**2) / 2) / 32768, 1.0)
    )


def test_levels_synthetic_sine():
    amplitude = 12000
    samples = [
        round(amplitude * math.sin(2 * math.pi * index / 128)) for index in range(128)
    ]
    rms, peak = calculate_levels(samples)
    assert rms == pytest.approx(amplitude / math.sqrt(2) / 32768, abs=1e-5)
    assert peak == pytest.approx(amplitude / 32768)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("AGCONOFF", 0),
        ("AGCMAXGAIN", 1000.0),
        ("AGCDESIREDLEVEL", 1e-08),
        ("AGCTIME", 0.1),
        ("HPFONOFF", 3),
        ("preset", "Noisy Environment / ASR"),
        ("led_mode", "mono"),
        ("led_brightness", 31),
        ("led_color", "#12ABEF"),
        ("vad_led", True),
    ],
)
def test_parameter_validation_accepts_documented_values(name, value):
    assert validate_parameter(name, value) == value


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("AGCONOFF", 2),
        ("AGCMAXGAIN", 0.5),
        ("AGCMAXGAIN", float("nan")),
        ("AGCTIME", 1.1),
        ("HPFONOFF", 4),
        ("preset", "Unknown"),
        ("led_mode", "rainbow"),
        ("led_brightness", 32),
        ("led_color", "12ABEF"),
        ("vad_led", 1),
    ],
)
def test_parameter_validation_rejects_invalid_values(name, value):
    with pytest.raises(ValueError):
        validate_parameter(name, value)


def test_presets_contain_every_tuning_parameter():
    standard_names = set(PRESETS["Standard"])
    for name, values in PRESETS.items():
        if name != "Custom":
            assert set(values) == standard_names


def test_float_readback_allows_xvf_fixed_point_rounding():
    assert readback_matches("AGCDESIREDLEVEL", 0.005, 0.00500001)
    assert not readback_matches("AGCDESIREDLEVEL", 0.005, 0.006)


def test_no_preset_requests_an_agctime_the_device_cannot_hold():
    for name, values in PRESETS.items():
        if name == "Custom":
            continue
        requested = values["AGCTIME"]
        assert readback_matches(
            "AGCTIME", requested, MEASURED_AGCTIME_READBACKS[requested]
        ), f"{name} requests an AGCTIME the device does not reproduce"


def test_agctime_tolerance_covers_the_measured_quantization():
    assert readback_matches("AGCTIME", 1.0, MEASURED_AGCTIME_READBACKS[1.0])
    assert not readback_matches("AGCTIME", 0.5, MEASURED_AGCTIME_READBACKS[0.5])


def test_tolerance_table_is_relative_and_absolute():
    assert readback_tolerance("AGCTIME") == (0.02, 1e-8)
    assert readback_tolerance("AGCMAXGAIN") == DEFAULT_READBACK_TOLERANCE
    assert readback_tolerance("unlisted") == DEFAULT_READBACK_TOLERANCE

    # Relative branch: 6.3e-6 of 31.6 is far more than the absolute epsilon.
    assert readback_matches("AGCMAXGAIN", 31.6, 31.6002)
    assert not readback_matches("AGCMAXGAIN", 31.6, 31.61)

    # Absolute branch: at the bottom of the range the relative tolerance is
    # 1e-13, so only the absolute epsilon can carry a read-back.
    assert readback_matches("AGCDESIREDLEVEL", 1e-08, 1.5e-08)
    assert not readback_matches("AGCDESIREDLEVEL", 1e-08, 3e-08)


def test_integer_readback_stays_exact():
    assert readback_matches("HPFONOFF", 2, 2)
    assert not readback_matches("HPFONOFF", 2, 1)


def test_mismatch_reason_names_parameter_and_both_values():
    reason = readback_mismatch_reason("AGCTIME", 0.5, MEASURED_AGCTIME_READBACKS[0.5])
    assert "AGCTIME" in reason
    assert "0.5" in reason
    assert "0.9685218567028642" in reason
    assert "0.02" in reason

    assert readback_mismatch_reason("HPFONOFF", 2, 1) == (
        "HPFONOFF read-back 1 != requested 2"
    )


@pytest.mark.parametrize("preset", [name for name in PRESETS if name != "Custom"])
def test_preset_applies_against_a_quantizing_device(preset):
    device = QuantizingDevice()

    readbacks, failure = apply_tuning_values(device, PRESETS[preset])

    assert failure is None
    assert set(readbacks) == set(TUNABLE_PARAMETERS)
    assert device.written["AGCTIME"] == 1.0
    assert readbacks["AGCTIME"] == MEASURED_AGCTIME_READBACKS[1.0]
    assert readbacks["HPFONOFF"] == PRESETS[preset]["HPFONOFF"]


def test_apply_reports_a_value_outside_its_tolerance():
    device = QuantizingDevice()

    readbacks, failure = apply_tuning_values(device, {"AGCTIME": 0.5})

    assert readbacks == {"AGCTIME": MEASURED_AGCTIME_READBACKS[0.5]}
    assert failure == readback_mismatch_reason(
        "AGCTIME", 0.5, MEASURED_AGCTIME_READBACKS[0.5]
    )


def test_apply_reports_a_failing_device_and_keeps_earlier_readbacks():
    device = BrokenDevice()

    readbacks, failure = apply_tuning_values(
        device, {"HPFONOFF": 2, "AGCONOFF": 0, "ECHOONOFF": 1}
    )

    assert readbacks == {"HPFONOFF": 2}
    assert failure == "Device write/read-back failed: usb timeout"
