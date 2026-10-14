from flask import abort, jsonify, request, Blueprint
from pib_hermes_config.live_interaction import personality_requests_actuation
from service import personality_service
from schema.personality_schema import (
    personality_schema,
    personalities_schema,
    upload_personality_schema,
    update_personality_schema,
)

bp = Blueprint("personality_controller", __name__)


def _reject_actuation_payload() -> None:
    payload = request.get_json(silent=True) or {}
    if isinstance(payload, dict) and personality_requests_actuation(payload):
        abort(400)


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
    _reject_actuation_payload()
    personality_dto = upload_personality_schema.load(request.json)
    personality = personality_service.create_personality(personality_dto)
    return personality_schema.dump(personality), 201


@bp.route("/<string:personality_id>", methods=["PUT"])
def update_personality(personality_id: str):
    _reject_actuation_payload()
    personality_dto = update_personality_schema.load(request.json)
    personality = personality_service.update_personality(
        personality_id, personality_dto
    )
    return personality_schema.dump(personality)


@bp.route("/<string:personality_id>/soul/append", methods=["POST"])
def append_soul_lesson(personality_id: str):
    payload = request.get_json(silent=True) or {}
    lesson = payload.get("lesson")
    if not isinstance(lesson, str):
        abort(400)
    lesson = lesson.strip()
    if not lesson or len(lesson) > 500:
        abort(400)
    personality = personality_service.append_soul_lesson(personality_id, lesson)
    return personality_schema.dump(personality)


@bp.route("/<string:personality_id>", methods=["DELETE"])
def delete_personality(personality_id: str):
    personality_service.delete_personality(personality_id)
    return "", 204
