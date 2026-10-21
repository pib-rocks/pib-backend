from flask import abort, jsonify, request, Blueprint, Response
from schema.chat_message_schema import (
    chat_message_post_schema,
    chat_message_schema,
    chat_messages_schema,
)
from schema.chat_schema import (
    chat_schema,
    chats_schema,
    upload_chat_schema,
    chat_messages_only_schema,
)
from service import chat_image_service, chat_service

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


@bp.route("/<string:chat_id>/first-token-latency", methods=["PUT"])
def put_first_token_latency(chat_id: str):
    """Record the first-token latency of this chat so an operator can read it."""
    payload = request.get_json(silent=True) or {}
    if "latencyMs" not in payload:
        abort(400)
    try:
        latency = float(payload["latencyMs"])
    except (TypeError, ValueError):
        abort(400)
    if latency < 0 or latency != latency:
        abort(400)
    chat = chat_service.record_first_token_latency(chat_id, latency)
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


@bp.route(
    "/<string:chat_id>/messages", defaults={"message_history": None}, methods=["GET"]
)
@bp.route("/<string:chat_id>/messages/<int:message_history>", methods=["GET"])
def get_messages_by_chat_id(chat_id: str, message_history: int | None = None):
    if message_history is None:
        chat = chat_service.get_chat(chat_id)
        return chat_messages_only_schema.dump(chat)
    else:
        messages = chat_service.get_message_history(chat_id, message_history)
        return jsonify({"messages": chat_messages_schema.dump(messages)})


@bp.route("/<string:chat_id>/messages/history/<int:message_history>", methods=["GET"])
def get_limited_amount_of_messages_by_chat(chat_id: str, message_history: int):
    messages = chat_service.get_message_history(chat_id, message_history)
    return jsonify({"messages": chat_messages_schema.dump(messages)})


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["DELETE"])
def delete_message(chat_id: str, message_id: str):
    # The service takes the chat as well: it scopes the message and deletes the
    # images captured during that turn with it (PC-119).
    chat_service.delete_message(chat_id, message_id)
    return "", 204


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["PUT"])
def patch_message(chat_id: str, message_id: str):
    chat_message_dto = chat_message_post_schema.load(request.json)
    chat_message = chat_service.update_chat_message(chat_message_dto, message_id)
    return chat_message_schema.dump(chat_message)


@bp.route("/<string:chat_id>/messages/<string:message_id>", methods=["GET"])
def get_message_by_chat_id_and_message_id(chat_id: str, message_id: str):
    message = chat_service.get_message(message_id)
    try:
        return chat_message_schema.dump(message)
    except Exception:
        abort(500)


# --- images captured by a tool during a turn (PC-119) -----------------------


@bp.route("/<string:chat_id>/messages/<string:message_id>/images", methods=["POST"])
def create_message_image(chat_id: str, message_id: str):
    """Store one captured image for one message.

    The message has to exist in this chat: an image that belongs to nothing would
    never be shown and never be deleted. The bytes live in a file, the answer is
    the reference the message meta carries.
    """
    chat_service.get_message_of_chat(chat_id, message_id)
    payload = request.json or {}
    try:
        image = chat_image_service.store(message_id, payload.get("imageBase64"))
    except chat_image_service.UnsupportedImage as error:
        return jsonify({"error": str(error)}), 400
    index = _image_index(message_id, image.id)
    reference = chat_image_service.public_reference(image, index)
    reference["url"] = _image_url(chat_id, message_id, index)
    return jsonify(reference), 201


@bp.route(
    "/<string:chat_id>/messages/<string:message_id>/images/<int:index>", methods=["GET"]
)
def get_message_image(chat_id: str, message_id: str, index: int):
    """Serve one stored image. Immutable: the id of an image never changes content."""
    image = chat_image_service.image_of(message_id, index)
    if image is None:
        abort(404)
    body = chat_image_service.read_bytes(image)
    if body is None:
        abort(404)
    response = Response(body, mimetype=image.mime_type)
    response.headers["Cache-Control"] = "public, max-age=31536000, immutable"
    response.headers["ETag"] = image.id
    return response


def _image_index(message_id: str, image_id: str) -> int:
    """Position of one image among its message's images, oldest first."""
    position = 0
    while True:
        candidate = chat_image_service.image_of(message_id, position)
        if candidate is None:
            return 0
        if candidate.id == image_id:
            return position
        position += 1


def _image_url(chat_id: str, message_id: str, index: int) -> str:
    """Server-relative URL; the chat prefixes its own API base."""
    return f"/voice-assistant/chat/{chat_id}/messages/{message_id}/images/{index}"
