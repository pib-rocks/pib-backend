from typing import Any, List

from pib_api.flask.app.app import db
from model.pose_model import Pose
from model.motor_position_model import MotorPosition


def get_all_poses() -> List[Pose]:
    return Pose.query.all()


def get_pose(pose_id: str) -> Pose:
    return Pose.query.filter(Pose.pose_id == pose_id).one()


def create_pose(pose_dto: dict[str, Any]) -> Pose:
    motor_position_dtos = pose_dto["motor_positions"]
    motor_positions = [_create_motor_position(dto) for dto in motor_position_dtos]
    pose = Pose(name=pose_dto["name"], motor_positions=motor_positions)
    db.session.add(pose)
    db.session.flush()
    return pose


def delete_pose(pose_id: str) -> None:
    db.session.delete(get_pose(pose_id))
    db.session.flush()


def _create_motor_position(motor_position_dto: dict[str, Any]) -> MotorPosition:
    motor_position = MotorPosition(
        position=motor_position_dto["position"],
        motor_name=motor_position_dto["motor_name"],
    )
    db.session.add(motor_position)
    return motor_position


def rename_pose(pose_id: str, pose_dto: dict[str, Any]) -> Pose:
    pose = get_pose(pose_id)
    pose.name = pose_dto["name"]
    db.session.flush()
    return pose
