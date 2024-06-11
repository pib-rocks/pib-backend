from service import pose_service
from schema.pose_schema import (
    poses_schema,
    pose_schema,
    pose_schema_motor_positions_only,
    create_pose_schema,
    pose_schema_name_only,
    pose_schema_without_motor_positions,
)
from flask import jsonify, request, Blueprint


bp = Blueprint("pose_controller", __name__)


@bp.route("", methods=["POST"])
def create_pose():
    pose_dto = create_pose_schema.load(request.json)
    pose = pose_service.create_pose(pose_dto)
    return pose_schema.dump(pose), 201


@bp.route("", methods=["GET"])
def get_all_poses():
    poses = pose_service.get_all_poses()
    return jsonify({"poses": poses_schema.dump(poses)})


@bp.route("/<string:pose_id>/motor-positions", methods=["GET"])
def get_motor_positions_of_pose(pose_id: str):
    pose = pose_service.get_pose(pose_id)
    return pose_schema_motor_positions_only.dump(pose)


@bp.route("/<string:pose_id>", methods=["DELETE"])
def delete_pose(pose_id: str):
    pose_service.delete_pose(pose_id)
    return "", 204


@bp.route("/<string:pose_id>", methods=["PATCH"])
def rename_pose(pose_id: str):
    pose_dto = pose_schema_name_only.load(request.json)
    pose = pose_service.rename_pose(pose_id, pose_dto)
    return pose_schema_without_motor_positions.dump(pose)
