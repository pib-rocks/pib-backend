import logging
from typing import Any, Dict, List, Tuple

from app.app import db
from default_pose_constants import STARTUP_POSE_NAME
from model.pose_model import Pose
from model.motor_position_model import MotorPosition
from model.motor_model import Motor

logger = logging.getLogger(__name__)

# pib_sdk.Write.move() takes degrees in [-90, 90] and refuses anything outside that range
# with a ValueError, so a stored pose above this limit cannot be played by a Blockly
# program: the generated apply_pose() dies in a traceback and the robot does not move.
POSITION_LIMIT = 9000


class PoseRefusedError(Exception):
    """The data model refuses this change to the pose (PR-1974).

    Deleting, renaming or updating a protected pose is a conflict with the pose's own
    state, not a server fault. A named condition so the app answers 409 Conflict instead
    of letting it fall through to the catch-all 500 handler - a caller has to be able to
    tell "you may not do this" from "the server broke".
    """


class PoseValidationError(Exception):
    """The request does not describe a valid update of a pose (PR-1974).

    Count and name mismatches between the request and the stored pose: a bad request
    (400), not a conflict and not a server error.
    """


def get_all_poses() -> List[Pose]:
    return Pose.query.filter().order_by(Pose.deletable.asc()).all()


def get_pose(pose_id: str) -> Pose:
    return Pose.query.filter(Pose.pose_id == pose_id).one()


def get_pose_by_name(pose_name: str) -> Pose:
    return Pose.query.filter(Pose.name == pose_name).one()


def _position_limits() -> Dict[str, Tuple[int, int]]:
    """Allowed centidegree range per motor, clamped to the playable +-90 degrees.

    A servo reports its position with its own tolerance, so a recorded joint can land a
    few centidegrees beyond the configured range. Storing that value makes the pose
    unplayable in Blockly; clamping keeps what the robot can actually reach.
    """
    limits: Dict[str, Tuple[int, int]] = {}
    for motor in Motor.query.all():
        low, high = -POSITION_LIMIT, POSITION_LIMIT
        if motor.rotation_range_min is not None:
            low = max(low, int(motor.rotation_range_min))
        if motor.rotation_range_max is not None:
            high = min(high, int(motor.rotation_range_max))
        if low > high:
            low, high = -POSITION_LIMIT, POSITION_LIMIT
        limits[motor.name] = (low, high)
    return limits


def clamp_position(
    motor_name: str, position: int, limits: Dict[str, Tuple[int, int]]
) -> int:
    bounds = limits.get(motor_name)
    if bounds is None:
        return position
    low, high = bounds
    clamped = max(low, min(high, int(position)))
    if clamped != position:
        logger.warning(
            "clamped pose position for %s from %s to %s (playable range %s..%s)",
            motor_name,
            position,
            clamped,
            low,
            high,
        )
    return clamped


def create_pose(pose_dto: dict[str, Any]) -> Pose:
    motor_position_dtos = pose_dto["motor_positions"]
    limits = _position_limits()
    motor_positions = [
        _create_motor_position(dto, limits) for dto in motor_position_dtos
    ]
    pose = Pose(name=pose_dto["name"], motor_positions=motor_positions)
    db.session.add(pose)
    db.session.flush()
    return pose


def delete_pose(pose_id: str) -> None:
    pose = get_pose(pose_id)
    if not pose.deletable:
        raise PoseRefusedError(f"Pose '{pose.name}' is not deletable")
    db.session.delete(pose)
    db.session.flush()


def _create_motor_position(
    motor_position_dto: dict[str, Any], limits: Dict[str, Tuple[int, int]]
) -> MotorPosition:
    motor_position = MotorPosition(
        position=clamp_position(
            motor_position_dto["motor_name"],
            motor_position_dto["position"],
            limits,
        ),
        motor_name=motor_position_dto["motor_name"],
    )
    db.session.add(motor_position)
    return motor_position


def rename_pose(pose_id: str, pose_dto: dict[str, Any]) -> Pose:
    pose = get_pose(pose_id)
    if not pose.deletable:
        raise PoseRefusedError(f"Pose '{pose.name}' cannot be renamed.")
    pose.name = pose_dto["name"]
    db.session.flush()
    return pose


def update_motor_positions_of_pose(pose_id: str, pose_dto: dict[str, Any]) -> Pose:
    pose = get_pose(pose_id)
    if not pose.deletable and pose.name != STARTUP_POSE_NAME:
        raise PoseRefusedError(f"Pose '{pose.name}' cannot be updated.")
    motor_position_dtos = pose_dto["motor_positions"]
    if len(motor_position_dtos) != len(pose.motor_positions):
        raise PoseValidationError(
            "Number of motor positions does not match existing pose."
        )
    motor_name_to_position = {
        mp["motor_name"]: mp["position"] for mp in motor_position_dtos
    }
    limits = _position_limits()
    for existing_mp in pose.motor_positions:
        if existing_mp.motor_name not in motor_name_to_position:
            raise PoseValidationError(
                f"Motor '{existing_mp.motor_name}' not found in update request."
            )
        existing_mp.position = clamp_position(
            existing_mp.motor_name,
            motor_name_to_position[existing_mp.motor_name],
            limits,
        )
    db.session.flush()
    return pose
