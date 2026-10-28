import logging
import time
from datetime import datetime
from typing import Any, List, Optional, Sequence

from werkzeug.exceptions import UnprocessableEntity

from app.app import db
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from provider_registry import MISSING_MODEL_CHAT_MESSAGE
from service import personality_service, provider_service

logger = logging.getLogger(__name__)


def _perf_ms(start: float) -> float:
    return (time.monotonic() - start) * 1000.0


def get_all_chats() -> List[Chat]:
    return Chat.query.all()


def get_chat(chat_id: str) -> Chat:
    return Chat.query.filter(Chat.chat_id == chat_id).one()


def get_message(message_id: str) -> ChatMessage:
    return ChatMessage.query.filter(ChatMessage.message_id == message_id).one()


def get_message_of_chat(chat_id: str, message_id: str) -> ChatMessage:
    """The message, if it belongs to this chat. Answers 404 for anything else.

    An image is attached to a message through the message id, so the pair has to
    be checked: an id from another chat would otherwise attach a picture to a
    conversation nobody opens, and it would never be deleted with its chat.
    """
    from flask import abort

    message = ChatMessage.query.filter(
        ChatMessage.message_id == message_id, ChatMessage.chat_id == chat_id
    ).one_or_none()
    if message is None:
        abort(404)
    return message


def get_message_history(chat_id: str, length: int) -> ChatMessage:
    return (
        ChatMessage.query.filter(ChatMessage.chat_id == chat_id)
        .order_by(ChatMessage.timestamp.desc())
        .limit(length)
        .all()
    )


def messages_etag(messages: Sequence[ChatMessage]) -> str:
    """Fingerprint of the full message set: last message id and count."""
    count = len(messages)
    last_id = messages[-1].message_id if messages else ""
    return f"{last_id}:{count}"


def _messages_after_since(
    messages: Sequence[ChatMessage], since: str
) -> list[ChatMessage]:
    for index, message in enumerate(messages):
        if message.message_id == since:
            return list(messages[index + 1 :])
    try:
        cutoff = datetime.fromisoformat(since.replace("Z", "+00:00"))
    except ValueError:
        return list(messages)
    return [message for message in messages if message.timestamp > cutoff]


def select_chat_messages(
    messages: Sequence[ChatMessage],
    *,
    since: Optional[str] = None,
    limit: Optional[int] = None,
) -> list[ChatMessage]:
    """Slice an ordered message list loaded through the chat model."""
    selected = list(messages)
    if since:
        selected = _messages_after_since(selected, since)
    if limit is not None:
        selected = selected[-limit:]
    return selected


def create_chat(chat_dto: Any) -> Chat:
    personality = personality_service.get_personality(chat_dto["personality_id"])
    # The model row is gone, so the chat cannot be given a model. It is not
    # started on a different one.
    if provider_service.find_provider(personality.provider_ref) is None:
        raise UnprocessableEntity(MISSING_MODEL_CHAT_MESSAGE)
    chat = Chat(topic=chat_dto["topic"], personality=personality)
    db.session.add(chat)
    db.session.flush()
    return chat


def update_chat(chat_id: str, chat_dto: Any) -> Chat:
    chat = get_chat(chat_id)
    chat.topic = chat_dto["topic"]
    db.session.flush()
    return chat


def record_first_token_latency(chat_id: str, latency_ms: float) -> Chat:
    """Store the latest first-token measurement for this chat."""
    chat = get_chat(chat_id)
    chat.first_token_latency_ms = float(latency_ms)
    db.session.flush()
    return chat


def delete_chat(chat_id: str) -> None:
    # Images belong to messages, so they go with the chat - row and file.
    from service import chat_image_service

    chat_image_service.delete_for_chat(chat_id)
    db.session.delete(get_chat(chat_id))
    db.session.flush()
    try:
        from public_api_client.hermes_agent_client import delete_session

        delete_session(chat_id)
    except Exception:
        # Best-effort: orphaned Hermes sessions can be pruned later.
        pass


def create_chat_message(chat_id: str, chat_message_dto: Any) -> ChatMessage:
    t0 = time.monotonic()
    logger.info(
        "[PERF_TRACE] API_ENTRY create_chat_message chat=%s elapsed_ms=0.00",
        chat_id,
    )
    chat = get_chat(chat_id)
    chat_message = ChatMessage(
        is_user=chat_message_dto["is_user"],
        content=chat_message_dto["content"],
        meta=chat_message_dto.get("meta"),
        chat=chat,
    )
    db.session.add(chat_message)
    db.session.flush()
    logger.info(
        "[PERF_TRACE] API_EXIT create_chat_message chat=%s elapsed_ms=%.2f",
        chat_id,
        _perf_ms(t0),
    )
    return chat_message


def update_chat_message(
    chat_message_dto: dict[str, Any], message_id: str
) -> ChatMessage:
    t0 = time.monotonic()
    logger.info(
        "[PERF_TRACE] API_ENTRY update_chat_message message=%s elapsed_ms=0.00",
        message_id,
    )
    chat_message = get_message(message_id)
    chat_message.content = chat_message_dto["content"]
    if chat_message_dto.get("meta") is not None:
        chat_message.meta = chat_message_dto["meta"]
    db.session.flush()
    logger.info(
        "[PERF_TRACE] API_EXIT update_chat_message message=%s elapsed_ms=%.2f",
        message_id,
        _perf_ms(t0),
    )
    return chat_message


def delete_message(chat_id: str, message_id: str) -> None:
    from service import chat_image_service

    chat_image_service.delete_for_message(message_id)
    db.session.delete(get_message(message_id))
    db.session.flush()
