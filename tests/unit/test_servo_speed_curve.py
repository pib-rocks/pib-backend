"""Unit tests for the measured servo speed curve (PR-2142)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_DIR) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_DIR))

from pib_motors.servo_speed_curve import (  # noqa: E402
    SERVO_FAMILY_FINGER,
    SERVO_FAMILY_STS3095,
    SPEED_CURVE_BY_FAMILY,
    STS_SPEED_MAX,
    STS_SPEED_MIN,
    curve_for_servo_id,
    family_for_servo_id,
    rate_for_speed,
    speed_for_path,
    speed_for_rate,
)


def test_family_follows_the_servo_id() -> None:
    assert family_for_servo_id(51) == SERVO_FAMILY_STS3095
    assert family_for_servo_id(19) == SERVO_FAMILY_STS3095
    assert family_for_servo_id(10) == SERVO_FAMILY_FINGER
    assert family_for_servo_id(36) == SERVO_FAMILY_FINGER
    assert curve_for_servo_id(51) == SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]


def test_the_measured_points_are_reproduced_exactly() -> None:
    """The curve must return the measurement at the measurement, or it is not the curve."""
    for family, points in SPEED_CURVE_BY_FAMILY.items():
        for setting, rate in points:
            assert rate_for_speed(setting, points) == pytest.approx(rate), family
            assert speed_for_rate(rate, points) == setting, family


def test_rate_rises_with_the_setting_and_saturates() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]
    rates = [
        rate_for_speed(setting, points)
        for setting in (0, 100, 200, 400, 700, 2000, 3400)
    ]
    assert rates == sorted(rates), rates
    # the curve flattens hard: 700 to 2000 gains only a few percent
    assert rate_for_speed(2000, points) < rate_for_speed(700, points) * 1.1


def test_inverse_never_asks_for_more_than_the_curve_can_do() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_FINGER]
    assert speed_for_rate(10_000.0, points) == 2000
    assert speed_for_rate(0.0, points) == 0
    # a negative rate is a direction, not a smaller speed: the magnitude decides
    assert speed_for_rate(-500.0, points) == speed_for_rate(500.0, points)


def test_direction_does_not_matter_for_the_magnitude() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]
    assert speed_for_rate(2000.0, points) == speed_for_rate(-2000.0, points)
    assert speed_for_path(-1500.0, points, margin=1.0) == speed_for_path(
        1500.0, points, margin=1.0
    )


def test_margin_increases_the_setting() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]
    without = speed_for_path(2000.0, points, margin=1.0)
    with_margin = speed_for_path(2000.0, points, margin=1.2)
    assert with_margin > without


def test_path_without_movement_keeps_the_minimum_setting() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]
    assert speed_for_path(0.0, points) == STS_SPEED_MIN
    # and a tiny movement is clamped, never set to zero: the joint has to hold its position
    assert speed_for_path(0.5, points, margin=1.0) == STS_SPEED_MIN


def test_result_stays_inside_the_setting_range() -> None:
    points = SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_STS3095]
    for path in (0.0, 1.0, 500.0, 5_000.0, 500_000.0):
        setting = speed_for_path(path, points, margin=1.2)
        assert STS_SPEED_MIN <= setting <= STS_SPEED_MAX, (path, setting)


def test_an_empty_curve_does_not_crash() -> None:
    assert rate_for_speed(500, ()) == 0.0
    assert speed_for_rate(500.0, ()) == 0
