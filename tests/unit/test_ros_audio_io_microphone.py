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
    AGCTIME_BLOCK_RATE_HZ,
    AGCTIME_READBACK_EPSILON_SECONDS,
    DEFAULT_READBACK_TOLERANCE,
    PRESETS,
    TUNABLE_PARAMETERS,
    agctime_coefficient,
    agctime_seconds,
    apply_tuning_values,
    parameter_from_readback,
    readback_matches,
    readback_mismatch_reason,
    readback_tolerance,
    validate_parameter,
)

# Read-backs measured on 192.168.1.172 with a ReSpeaker Mic Array v2.0. They
# are coefficients of the 62.5 Hz block rate, not seconds.
MEASURED_AGCTIME_READBACKS = {
    0.5: 0.9685218567028642,
    1.0: 0.9841422392055392,
}
# Both measured read-backs sit this far above exp(-1 / (62.5 * seconds)).
MEASURED_COEFFICIENT_RESIDUAL = 1.5e-05


class CoefficientDevice:
    """Device double answering AGCTIME the way the XVF3000 was measured to.

    ``held_agctime_seconds`` models a device that stores a ramp time other than
    the requested one.
    """

    def __init__(self, held_agctime_seconds=None):
        self.written = {}
        self._held_agctime_seconds = held_agctime_seconds

    def write(self, name, value):
        self.written[name] = value

    def read(self, name):
        value = self.written[name]
        if name == "AGCTIME":
            if self._held_agctime_seconds is not None:
                value = self._held_agctime_seconds
            return agctime_coefficient(value) + MEASURED_COEFFICIENT_RESIDUAL
        return value


class SaturatedDevice(CoefficientDevice):
    """Device double whose AGCTIME register reads back outside (0, 1)."""

    def read(self, name):
        if name == "AGCTIME":
            return 1.0
        return super().read(name)


class BrokenDevice(CoefficientDevice):
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


def test_noisy_preset_keeps_its_intended_half_second_ramp():
    assert PRESETS["Noisy Environment / ASR"]["AGCTIME"] == 0.5


def test_block_rate_is_the_processing_rate_of_the_device():
    assert AGCTIME_BLOCK_RATE_HZ == 16000 / 256 == 62.5


@pytest.mark.parametrize("requested", sorted(MEASURED_AGCTIME_READBACKS))
def test_measured_readbacks_follow_the_block_rate_model(requested):
    measured = MEASURED_AGCTIME_READBACKS[requested]

    # 1.49e-05 at 1.0 s and 1.53e-05 at 0.5 s: one systematic residual.
    assert measured - agctime_coefficient(requested) == pytest.approx(
        MEASURED_COEFFICIENT_RESIDUAL, abs=5e-07
    )
    assert agctime_seconds(measured) == pytest.approx(
        requested, abs=AGCTIME_READBACK_EPSILON_SECONDS
    )


@pytest.mark.parametrize("seconds", [0.1, 0.25, 0.5, 1.0])
def test_agctime_converts_from_seconds_and_back(seconds):
    coefficient = agctime_coefficient(seconds)

    assert 0.0 < coefficient < 1.0
    assert agctime_seconds(coefficient) == pytest.approx(seconds, abs=1e-12)


def test_predicted_quarter_second_coefficient():
    assert agctime_coefficient(0.25) == pytest.approx(0.938005, abs=1e-06)


@pytest.mark.parametrize("coefficient", [0.0, 1.0, -0.5, 1.5])
def test_agctime_rejects_a_coefficient_no_ramp_time_maps_to(coefficient):
    with pytest.raises(ValueError):
        agctime_seconds(coefficient)


def test_agctime_rejects_a_non_positive_ramp_time():
    with pytest.raises(ValueError):
        agctime_coefficient(0.0)


def test_only_agctime_is_converted_out_of_the_device_unit():
    assert parameter_from_readback(
        "AGCTIME", MEASURED_AGCTIME_READBACKS[0.5]
    ) == pytest.approx(0.5, abs=AGCTIME_READBACK_EPSILON_SECONDS)
    assert parameter_from_readback("AGCMAXGAIN", 31.6) == 31.6
    assert parameter_from_readback("HPFONOFF", 2) == 2


def test_agctime_epsilon_covers_the_residual_but_not_a_wrong_ramp_time():
    assert readback_matches(
        "AGCTIME", 1.0, agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    )
    assert readback_matches(
        "AGCTIME", 0.5, agctime_seconds(MEASURED_AGCTIME_READBACKS[0.5])
    )

    # The 1.0 s coefficient answering a 0.5 s write stays a rejection.
    assert not readback_matches(
        "AGCTIME", 0.5, agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    )
    # A ramp time 5 ms off is more than the conversion residual explains.
    assert not readback_matches("AGCTIME", 0.5, 0.505)


def test_tolerance_table_is_relative_and_absolute():
    assert readback_tolerance("AGCTIME") == (
        DEFAULT_READBACK_TOLERANCE[0],
        AGCTIME_READBACK_EPSILON_SECONDS,
    )
    assert AGCTIME_READBACK_EPSILON_SECONDS == 2e-3
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
    held = agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    reason = readback_mismatch_reason(
        "AGCTIME", 0.5, held, MEASURED_AGCTIME_READBACKS[1.0]
    )
    assert "AGCTIME" in reason
    assert "0.5" in reason
    assert str(held) in reason
    assert "0.9841422392055392" in reason
    assert "0.002" in reason

    assert readback_mismatch_reason("HPFONOFF", 2, 1) == (
        "HPFONOFF read-back 1 != requested 2"
    )


@pytest.mark.parametrize("preset", [name for name in PRESETS if name != "Custom"])
def test_preset_applies_against_a_device_answering_in_coefficients(preset):
    device = CoefficientDevice()

    readbacks, failure = apply_tuning_values(device, PRESETS[preset])

    assert failure is None
    assert set(readbacks) == set(TUNABLE_PARAMETERS)
    requested = PRESETS[preset]["AGCTIME"]
    assert device.written["AGCTIME"] == requested
    assert readbacks["AGCTIME"] == pytest.approx(
        requested, abs=AGCTIME_READBACK_EPSILON_SECONDS
    )
    assert readbacks["HPFONOFF"] == PRESETS[preset]["HPFONOFF"]


def test_apply_rejects_a_ramp_time_the_device_does_not_store():
    device = CoefficientDevice(held_agctime_seconds=1.0)

    readbacks, failure = apply_tuning_values(device, {"AGCTIME": 0.5})

    held = readbacks["AGCTIME"]
    assert held == pytest.approx(1.0, abs=AGCTIME_READBACK_EPSILON_SECONDS)
    assert failure == readback_mismatch_reason(
        "AGCTIME", 0.5, held, agctime_coefficient(1.0) + MEASURED_COEFFICIENT_RESIDUAL
    )
    assert "AGCTIME" in failure
    assert "0.5" in failure


def test_apply_rejects_a_readback_that_is_not_a_ramp_time_at_all():
    device = SaturatedDevice()

    readbacks, failure = apply_tuning_values(device, {"AGCTIME": 0.5})

    assert readbacks == {}
    assert "AGCTIME" in failure
    assert "0.5" in failure
    assert "1.0" in failure


def test_apply_reports_a_failing_device_and_keeps_earlier_readbacks():
    device = BrokenDevice()

    readbacks, failure = apply_tuning_values(
        device, {"HPFONOFF": 2, "AGCONOFF": 0, "ECHOONOFF": 1}
    )

    assert readbacks == {"HPFONOFF": 2}
    assert failure == "Device write/read-back failed: usb timeout"
