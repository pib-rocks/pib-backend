"""Unit tests for ROS radian ↔ centidegree conversion (PR-2048)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_ROOT = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_ROOT) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_ROOT))

from pib_motors.units import CENTIDEG_PER_RADIAN, centideg_to_rad, rad_to_centideg


def test_golden_pairs() -> None:
    assert rad_to_centideg(0.0) == 0
    assert centideg_to_rad(0) == pytest.approx(0.0)
    assert rad_to_centideg(math.pi / 2.0) == 9000
    assert centideg_to_rad(9000) == pytest.approx(math.pi / 2.0)
    assert rad_to_centideg(-math.pi / 2.0) == -9000
    assert centideg_to_rad(-9000) == pytest.approx(-math.pi / 2.0)


def test_centideg_per_radian_constant() -> None:
    assert CENTIDEG_PER_RADIAN == pytest.approx(18000.0 / math.pi)


def test_round_trip_over_sweep() -> None:
    for centideg in range(-9000, 9001, 37):
        radians = centideg_to_rad(centideg)
        assert rad_to_centideg(radians) == centideg


def test_rad_to_centideg_rejects_non_finite() -> None:
    with pytest.raises(ValueError, match="finite"):
        rad_to_centideg(float("nan"))
