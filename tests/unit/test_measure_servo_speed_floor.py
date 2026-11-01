"""Pure helpers for PR-2153 low-speed calibration."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_DIR) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_DIR))

from pib_motors.servo_speed_measurement import (  # noqa: E402
    assert_recording_valid,
    centideg_delta_for_ticks,
    centideg_to_rad,
    cruise_slope_centideg_per_sec,
    position_step_ratio,
    rad_to_centideg,
    suggest_continuous_floor_cdps,
    summarize_cruise_rates,
)


def test_cruise_slope_uses_the_middle_of_the_move():
    samples = [(0.0, 0.0), (1.0, 0.1), (2.0, 0.2), (3.0, 0.3)]
    rate = cruise_slope_centideg_per_sec(samples)
    expected = rad_to_centideg(0.3) / 3.0
    assert rate == pytest.approx(expected, rel=0.05)


def test_assert_recording_valid_rejects_all_zeros():
    err = assert_recording_valid([(0.0, 0.0), (1.0, 0.0)], label="t")
    assert err is not None
    assert "zero" in err.lower()


def test_assert_recording_valid_accepts_motion():
    samples = [(0.0, 0.0), (1.0, centideg_to_rad(100.0))]
    assert assert_recording_valid(samples, label="t") is None


def test_step_ratio_flags_uneven_motion():
    # mostly equal steps with one jump - the case a stuttering slow move produces. Two steps
    # alone cannot show it: the median of two values is one of them, so the ratio stays ~1.
    samples = [(float(i), 0.001 * i) for i in range(6)] + [(6.0, 0.006 + 0.05)]
    stats = position_step_ratio(samples)
    assert stats["largest_to_median_ratio"] > 2.0


def test_step_ratio_of_a_uniform_move_is_one():
    samples = [(float(i), 0.002 * i) for i in range(6)]
    stats = position_step_ratio(samples)
    assert stats["largest_to_median_ratio"] == pytest.approx(1.0)


def test_centideg_delta_for_ticks_matches_encoder():
    assert centideg_delta_for_ticks(1) == pytest.approx(36000.0 / 4096.0)


def test_summarize_cruise_rates_reports_spread():
    summary = summarize_cruise_rates([100.0, 120.0, 110.0])
    assert summary["mean_cdps"] == pytest.approx(110.0)
    assert summary["stdev_cdps"] > 0.0


def test_suggest_continuous_floor_picks_lowest_smooth_setting():
    table = {
        50: {"mean_cdps": 500.0, "worst_step_ratio": 3.0},
        100: {"mean_cdps": 800.0, "worst_step_ratio": 1.5},
    }
    setting, cdps = suggest_continuous_floor_cdps(table)
    assert setting == 100
    assert cdps == pytest.approx(800.0)
