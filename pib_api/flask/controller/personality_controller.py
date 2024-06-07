from service import personality_service
from schema.personality_schema import (
    personality_schema,
    personalities_schema,
    upload_personality_schema,
)
from app.app import db
from flask import abort, jsonify, request, Blueprint

bp = Blueprint("personality_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_personalities():
    personalities = personality_service.get_all_personalities()
    personalities_dto = personalities_schema.dump(personalities)
    try:
        return jsonify({"voiceAssistantPersonalities": personalities_dto})
    except Exception:
        abort(500)


@bp.route("/<string:personality_id>", methods=["GET"])
def get_personality(personality_id: str):
    personality = personality_service.get_personality(personality_id)
    try:
        return personality_schema.dump(personality)
    except Exception:
        abort(500)


@bp.route("", methods=["POST"])
def create_personality():
    personality_dto = upload_personality_schema.load(request.json)
    personality = personality_service.create_personality(personality_dto)
    db.session.commit()
    try:
        return personality_schema.dump(personality), 201
    except Exception:
        abort(500)


@bp.route("/<string:personality_id>", methods=["PUT"])
def update_personality(personality_id: str):
    personality_dto = upload_personality_schema.load(request.json)
    personality = personality_service.update_personality(
        personality_id, personality_dto
    )
    db.session.commit()
    try:
        return personality_schema.dump(personality)
    except Exception:
        abort(500)


@bp.route("/<string:personality_id>", methods=["DELETE"])
def delete_personality(personality_id: str):
    personality_service.delete_personality(personality_id)
    db.session.commit()
    return "", 204
