from flask import jsonify, request, Blueprint
from schema.chat_message_schema import (
    chat_message_post_schema,
    chat_message_schema,
)
from schema.chat_schema import (
    chat_schema,
    chats_schema,
    upload_chat_schema,
    chat_messages_only_schema,
)
from service import chat_service

bp = Blueprint("chat_controller", __name__)


@bp.route("", methods=["POST"])
def create_chat():
    chat_dto = upload_chat_schema.load(request.json)
    chat = chat_service.create_chat(chat_dto)
    return chat_schema.dump(chat), 201


@bp.route("", methods=["GET"])
def get_all_chats():
    chats = chat_service.get_all_chats()
    return jsonify({"voiceAssistantChats": chats_schema.dump(chats)})


@bp.route("/<string:chat_id>", methods=["GET"])
def get_chat_by_id(chat_id: str):
    chat = chat_service.get_chat(chat_id)
    return chat_schema.dump(chat)


@bp.route("/<string:chat_id>", methods=["PUT"])
def update_chat(chat_id: str):
    chat_dto = upload_chat_schema.load(request.json)
    chat = chat_service.update_chat(chat_id, chat_dto)
    return chat_schema.dump(chat)


@bp.route("/<string:chat_id>", methods=["DELETE"])
def delete_chat(chat_id: str):
    chat_service.delete_chat(chat_id)
    return "", 204


@bp.route("/<string:chat_id>/messages", methods=["POST"])
def create_message(chat_id: str):
    chat_message_dto = chat_message_post_schema.load(request.json)
    chat_message = chat_service.create_chat_message(chat_id, chat_message_dto)
    return chat_message_schema.dump(chat_message), 201


@bp.route("/<string:chat_id>/messages", methods=["GET"])
def get_messages_by_chat_id(chat_id: str):
    chat = chat_service.get_chat(chat_id)
    return chat_messages_only_schema.dump(chat)


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["DELETE"])
def delete_message(chat_id: str, message_id: str):
    chat_service.delete_message(chat_id, message_id)
    return "", 204


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["PUT"])
def patch_message(chat_id: str, message_id: str):
    chat_message_dto = chat_message_post_schema.load(request.json)
    chat_message = chat_service.update_chat_message(
        chat_message_dto, chat_id, message_id
    )
    return chat_message_schema.dump(chat_message)


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["GET"])
def get_message_by_chat_id_and_message_id(chat_id: str, message_id: str):
    message = chat_service.get_message(chat_id, message_id)
    try:
        return chat_message_schema.dump(message)
    except Exception:
        abort(500)
