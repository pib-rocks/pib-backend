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
    PRESETS,
    readback_matches,
    validate_parameter,
)


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
    assert readback_matches(0.005, 0.00500001)
    assert not readback_matches(0.005, 0.006)
