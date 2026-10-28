"""JointTrajectory command parsing and validation (ROS-runtime-free).

Wire contract: ``joint_names[i]`` pairs with ``positions[i]`` on each trajectory point.
A single point is an immediate target command; multiple points are a timed trajectory.
The legacy pib zip shape (N names with N single-position points) is rejected.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping, Sequence, Tuple

MotorOutcome = Tuple[str, bool, str, str]  # name, successful, error_code, detail


def is_legacy_zip_shape(jt: Any) -> bool:
    """True for the retired N-names / N-single-position-points layout."""
    names = list(getattr(jt, "joint_names", []) or [])
    points = list(getattr(jt, "points", []) or [])
    if len(names) <= 1:
        return False
    if len(points) != len(names):
        return False
    return all(len(getattr(p, "positions", []) or []) <= 1 for p in points)


def iter_single_point_centideg_targets(
    jt: Any,
) -> Iterable[Tuple[str, int]]:
    """Yield ``(joint_name, centidegree)`` from a single-point trajectory."""
    names = list(getattr(jt, "joint_names", []) or [])
    points = list(getattr(jt, "points", []) or [])
    if not names or not points:
        return
    positions = list(getattr(points[0], "positions", []) or [])
    if len(positions) < len(names):
        raise ValueError(
            f"trajectory point has {len(positions)} positions for "
            f"{len(names)} joint names"
        )
    for index, name in enumerate(names):
        yield name, int(round(float(positions[index])))


def _classify_validation_error(message: str) -> str:
    lowered = message.lower()
    if "unknown joint" in lowered:
        return "unknown_motor"
    if "finite" in lowered:
        return "not_finite"
    if "outside allowed range" in lowered:
        return "out_of_range"
    return "out_of_range"


def validate_radians_trajectory(
    jt: Any,
    name_to_motors: Mapping[str, Sequence[Any]],
    rad_to_centideg_for_joint: Any,
) -> tuple[dict[str, Tuple[str, str]] | None, list[MotorOutcome]]:
    """Validate every joint on every point before any motor command.

    Returns ``(errors_by_joint_name, outcomes)``. On success ``errors`` is ``None`` and
    every outcome is successful. On failure ``errors`` maps joint names to
    ``(error_code, detail)`` and outcomes list one entry per ``joint_names`` order.
    """
    names = list(getattr(jt, "joint_names", []) or [])
    points = list(getattr(jt, "points", []) or [])
    errors: dict[str, Tuple[str, str]] = {}

    for name in names:
        if name not in name_to_motors:
            detail = f"unknown joint name '{name}'"
            errors[name] = ("unknown_motor", detail)

    for point in points:
        positions = list(getattr(point, "positions", []) or [])
        if len(positions) < len(names):
            detail = (
                f"trajectory point has {len(positions)} positions for "
                f"{len(names)} joint names"
            )
            for name in names:
                if name not in errors:
                    errors[name] = ("out_of_range", detail)
            continue
        for index, name in enumerate(names):
            if name in errors:
                continue
            radians = float(positions[index])
            if name not in name_to_motors:
                continue
            if not math.isfinite(radians):
                errors[name] = (
                    "not_finite",
                    f"joint '{name}' position must be finite, got {radians}",
                )
                continue
            try:
                rad_to_centideg_for_joint(name, radians, name_to_motors)
            except ValueError as exc:
                errors[name] = (_classify_validation_error(str(exc)), str(exc))

    if errors:
        outcomes: list[MotorOutcome] = []
        for name in names:
            if name in errors:
                code, detail = errors[name]
                outcomes.append((name, False, code, detail))
            else:
                outcomes.append((name, False, "", "command rejected"))
        return errors, outcomes

    success_outcomes = [(name, True, "", "") for name in names]
    return None, success_outcomes


def legacy_zip_outcomes(joint_names: Sequence[str]) -> list[MotorOutcome]:
    detail = (
        "legacy zip trajectory shape is not supported; use one point with "
        "one position per joint name"
    )
    return [(name, False, "legacy_shape_rejected", detail) for name in joint_names]


def outcomes_to_success(outcomes: Sequence[MotorOutcome]) -> bool:
    return bool(outcomes) and all(item[1] for item in outcomes)
