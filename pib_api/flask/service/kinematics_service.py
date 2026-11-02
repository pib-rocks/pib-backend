"""Inverse kinematics for the console: pib-sdk behind a small HTTP contract.

The 3D tab drags a hand and asks for joint angles. The solver lives in pib-sdk - this module
only feeds it, converts units and turns its typed failures into an answer the drag loop can use
(a failure is a normal reply, not a 500: the pointer keeps moving).

Units: the API speaks metres in the REP-103 base frame; pib-sdk speaks millimetres.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from typing import Any

SIDES = ("left", "right")


class KinematicsRequestError(ValueError):
    """Malformed request - the caller sent something the solver cannot be asked."""


_CHAINS: dict[str, Any] = {}


def _chain(side: str):
    if side not in SIDES:
        raise KinematicsRequestError(f"side must be one of {SIDES}")
    if side not in _CHAINS:
        from pib_sdk import ArmKinematics

        # ArmKinematics carries the model bundled with pib-sdk; no checkout, no ROS.
        _CHAINS[side] = ArmKinematics(side)
    return _CHAINS[side]


def _sdk_errors():
    from pib_sdk import kinematics as k

    return k


def _failure_types() -> tuple[type[BaseException], ...]:
    """The four failures the solver reports as a result instead of an exception."""
    k = _sdk_errors()
    return (
        k.InverseKinematicsWorkspaceError,
        k.InverseKinematicsBehindBodyError,
        k.InverseKinematicsJointLimitError,
        k.InverseKinematicsConvergenceError,
    )


def _solve(
    side: str,
    xyz_mm: Sequence[float],
    seed_rad: Sequence[float] | None,
    restarts: int = 50,
    max_iterations: int = 200,
):
    """One solve. Returns (joints_rad, stats). Raises the pib-sdk errors unchanged.

    `restarts` and `max_iterations` are the budget: the drag loop asks for a fast answer
    (few restarts), a one-off call can afford the patient one. A target that does not converge
    used to run the full 50 x 200 search - 4.5 s on the robot, long after the console's timeout.
    """
    chain = _chain(side)
    joints = chain.inverse(
        xyz=xyz_mm,
        initial_guess_rad=seed_rad,
        respect_limits=True,
        restarts=restarts,
        max_iterations=max_iterations,
    )
    stats = getattr(chain, "last_inverse_stats", {}) or {}
    return [float(v) for v in joints], stats


def _closest_reachable(side: str, xyz_mm: Sequence[float], seed_rad, restarts: int = 2):
    """Walk the ray from the base to the target and keep the last reachable point.

    The UI shows this as "the hand stops at the edge" instead of a rejection. A coarse scan finds
    the first reachable sample; four bisections refine it. Near the base a target can also be
    "behind the body" or inside the torso, so a monotone bisection over mixed failure kinds is not
    enough - a scan is honest about that.
    """
    best_joints, best_point, best_f = None, None, None
    # Eight samples, three refinements: each call costs a solve, and a drag cannot wait.
    for step in range(8, 0, -1):
        fraction = step / 8
        point = [c * fraction for c in xyz_mm]
        try:
            joints, _stats = _solve(side, point, seed_rad, restarts=restarts, max_iterations=60)
        except _failure_types():
            continue
        best_joints, best_point, best_f = joints, point, fraction
        break

    if best_point is None:
        return None, None
    lo, hi = float(best_f or 0.0), min(1.0, float(best_f or 0.0) + 1 / 8)
    for _ in range(3):
        mid = (lo + hi) / 2
        point = [c * mid for c in xyz_mm]
        try:
            joints, _stats = _solve(side, point, seed_rad, restarts=restarts, max_iterations=60)
        except _failure_types():
            hi = mid
            continue
        best_joints, best_point = joints, point
        lo = mid
    return best_joints, best_point


def inverse(
    side: str,
    xyz_m: Sequence[float],
    rpy_rad: Sequence[float] | None = None,
    seed_rad: Sequence[float] | None = None,
    restarts: int = 50,
    max_iterations: int = 200,
) -> dict[str, Any]:
    """Solve one target. Always returns a dict with a ``reach`` report."""
    if len(xyz_m) != 3:
        raise KinematicsRequestError("xyzM must have three components")
    xyz_mm = [float(c) * 1000.0 for c in xyz_m]
    started = time.perf_counter()

    try:
        joints, stats = _solve(side, xyz_mm, seed_rad, restarts, max_iterations)
        return _answer(
            side,
            joints,
            ok=True,
            distance_mm=0.0,
            blocked_by=None,
            stats=stats,
            started=started,
        )
    except _failure_types() as error:
        mapped = _map_failure(side, error, xyz_mm, seed_rad, restarts)
        if mapped is None or mapped.get("joints") is None:
            # Nothing on the ray is reachable (the target is behind the body or inside the torso).
            # Hand back no joints at all: the console keeps the current pose instead of inventing
            # one, and the readout says why.
            return _answer(
                side,
                None,
                ok=False,
                distance_mm=sum(c * c for c in xyz_mm) ** 0.5,
                blocked_by=_blocked_by(error),
                stats={},
                started=started,
            )
        return _answer(
            side,
            mapped["joints"],
            ok=False,
            distance_mm=mapped["distance_mm"],
            blocked_by=mapped["blocked_by"],
            stats=mapped.get("stats") or {},
            started=started,
        )


def _blocked_by(error: Exception) -> str:
    k = _sdk_errors()
    if isinstance(error, k.InverseKinematicsBehindBodyError):
        return "behind_body"
    if isinstance(error, k.InverseKinematicsJointLimitError):
        return "joint_limit"
    if isinstance(error, k.InverseKinematicsConvergenceError):
        return "convergence"
    return "workspace"


def _map_failure(side: str, error: Exception, xyz_mm, seed_rad, restarts: int = 2):
    k = _sdk_errors()
    if isinstance(
        error, (k.InverseKinematicsWorkspaceError, k.InverseKinematicsBehindBodyError)
    ):
        blocked = (
            "behind_body"
            if isinstance(error, k.InverseKinematicsBehindBodyError)
            else "workspace"
        )
        joints, point = _closest_reachable(side, xyz_mm, seed_rad, restarts)
        if joints is None or point is None:
            return None
        distance = sum((a - b) ** 2 for a, b in zip(xyz_mm, point)) ** 0.5
        return {"joints": joints, "distance_mm": distance, "blocked_by": blocked}
    if isinstance(error, k.InverseKinematicsJointLimitError):
        return _constraint_failure(side, xyz_mm, seed_rad, "joint_limit", restarts)
    if isinstance(error, k.InverseKinematicsConvergenceError):
        return _constraint_failure(side, xyz_mm, seed_rad, "convergence", restarts)
    return None


def _constraint_failure(
    side: str, xyz_mm, seed_rad, blocked_by: str, restarts: int = 2
):
    joints, point = _closest_reachable(side, xyz_mm, seed_rad, restarts)
    if joints is None or point is None:
        return None
    distance = sum((a - b) ** 2 for a, b in zip(xyz_mm, point)) ** 0.5
    return {"joints": joints, "distance_mm": distance, "blocked_by": blocked_by}


def _answer(
    side, joints, *, ok, distance_mm, blocked_by, stats, started
) -> dict[str, Any]:
    chain = _chain(side)
    return {
        "jointNames": list(chain.joint_names),
        "jointsRad": joints,
        "reach": {
            "ok": ok,
            "distanceMm": round(float(distance_mm), 2),
            "blockedBy": blocked_by,
        },
        "iterations": int(stats.get("iterations", 0) or 0),
        "elapsedMs": round((time.perf_counter() - started) * 1000.0, 2),
    }
