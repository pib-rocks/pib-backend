from service import personality_service
from schema.personality_schema import (
    personality_schema,
    personalities_schema,
    upload_personality_schema,
)
from flask import jsonify, request, Blueprint

bp = Blueprint("personality_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_personalities():
    personalities = personality_service.get_all_personalities()
    personalities_dto = personalities_schema.dump(personalities)
    return jsonify({"voiceAssistantPersonalities": personalities_dto})


@bp.route("/<string:personality_id>", methods=["GET"])
def get_personality(personality_id: str):
    personality = personality_service.get_personality(personality_id)
    return personality_schema.dump(personality)


@bp.route("", methods=["POST"])
def create_personality():
    personality_dto = upload_personality_schema.load(request.json)
    personality = personality_service.create_personality(personality_dto)
    return personality_schema.dump(personality), 201


@bp.route("/<string:personality_id>", methods=["PUT"])
def update_personality(personality_id: str):
    personality_dto = upload_personality_schema.load(request.json)
    personality = personality_service.update_personality(
        personality_id, personality_dto
    )
    return personality_schema.dump(personality)


@bp.route("/<string:personality_id>", methods=["DELETE"])
def delete_personality(personality_id: str):
    personality_service.delete_personality(personality_id)
    return "", 204
