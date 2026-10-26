"""Unit tests for joint trajectory command parsing and validation (PR-2076)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_ROOT = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_ROOT) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_ROOT))

from pib_motors.joint_trajectory_command import (
    is_legacy_zip_shape,
    iter_single_point_centideg_targets,
    legacy_zip_outcomes,
    validate_radians_trajectory,
)
from pib_motors.units import centideg_to_rad


class FakePoint:
    def __init__(self, positions: list[float]) -> None:
        self.positions = list(positions)


class FakeJointTrajectory:
    def __init__(self, joint_names: list[str], points: list[FakePoint]) -> None:
        self.joint_names = joint_names
        self.points = points


class FakeMotor:
    def __init__(
        self,
        name: str,
        *,
        rotation_range_min: int = -9000,
        rotation_range_max: int = 9000,
    ) -> None:
        self.name = name
        self.rotation_range_min = rotation_range_min
        self.rotation_range_max = rotation_range_max


def _rad_to_centideg_for_joint(
    joint_name: str, radians: float, name_to_motors: dict[str, list[FakeMotor]]
) -> float:
    from pib_motors.units import rad_to_centideg

    if joint_name not in name_to_motors:
        raise ValueError(f"unknown joint name '{joint_name}'")
    if not math.isfinite(radians):
        raise ValueError(f"joint '{joint_name}' position must be finite, got {radians}")
    centideg = rad_to_centideg(radians)
    motor = name_to_motors[joint_name][0]
    if centideg < motor.rotation_range_min or centideg > motor.rotation_range_max:
        raise ValueError(
            f"joint '{joint_name}' position {radians} rad ({centideg} centideg) "
            f"is outside allowed range [{motor.rotation_range_min}, "
            f"{motor.rotation_range_max}] centidegrees"
        )
    return float(centideg)


def test_single_point_six_positions_yields_six_targets() -> None:
    names = [f"motor_{index}" for index in range(6)]
    jt = FakeJointTrajectory(names, [FakePoint([10.0, 20.0, 30.0, 40.0, 50.0, 60.0])])
    targets = list(iter_single_point_centideg_targets(jt))
    assert targets == [
        ("motor_0", 10),
        ("motor_1", 20),
        ("motor_2", 30),
        ("motor_3", 40),
        ("motor_4", 50),
        ("motor_5", 60),
    ]


def test_legacy_zip_shape_detected() -> None:
    jt = FakeJointTrajectory(
        ["a", "b", "c"],
        [FakePoint([1.0]), FakePoint([2.0]), FakePoint([3.0])],
    )
    assert is_legacy_zip_shape(jt) is True


def test_legacy_zip_outcomes() -> None:
    outcomes = legacy_zip_outcomes(["a", "b"])
    assert len(outcomes) == 2
    assert all(code == "legacy_shape_rejected" for _, _, code, _ in outcomes)


def test_invalid_middle_joint_rejects_all_without_partial_codes() -> None:
    lookup = {
        "good": [FakeMotor("good")],
        "bad": [FakeMotor("bad")],
        "also_good": [FakeMotor("also_good")],
    }
    jt = FakeJointTrajectory(
        ["good", "bad", "also_good"],
        [
            FakePoint(
                [
                    centideg_to_rad(1000),
                    float("nan"),
                    centideg_to_rad(2000),
                ]
            )
        ],
    )
    errors, outcomes = validate_radians_trajectory(
        jt, lookup, _rad_to_centideg_for_joint
    )
    assert errors is not None
    assert outcomes[0] == ("good", False, "", "command rejected")
    assert outcomes[1][0] == "bad"
    assert outcomes[1][1] is False
    assert outcomes[1][2] == "not_finite"
    assert outcomes[2] == ("also_good", False, "", "command rejected")
