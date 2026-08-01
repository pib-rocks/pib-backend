import logging
import time
from typing import Any, List

from app.app import db
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from service import personality_service

logger = logging.getLogger(__name__)


def _perf_ms(start: float) -> float:
    return (time.monotonic() - start) * 1000.0


def get_all_chats() -> List[Chat]:
    return Chat.query.all()


def get_chat(chat_id: str) -> Chat:
    return Chat.query.filter(Chat.chat_id == chat_id).one()


def get_message(message_id: str) -> ChatMessage:
    return ChatMessage.query.filter(ChatMessage.message_id == message_id).one()


def get_message_history(chat_id: str, length: int) -> ChatMessage:
    return (
        ChatMessage.query.filter(ChatMessage.chat_id == chat_id)
        .order_by(ChatMessage.timestamp.desc())
        .limit(length)
        .all()
    )


def create_chat(chat_dto: Any) -> Chat:
    personality = personality_service.get_personality(chat_dto["personality_id"])
    chat = Chat(topic=chat_dto["topic"], personality=personality)
    db.session.add(chat)
    db.session.flush()
    return chat


def update_chat(chat_id: str, chat_dto: Any) -> Chat:
    chat = get_chat(chat_id)
    chat.topic = chat_dto["topic"]
    db.session.flush()
    return chat


def delete_chat(chat_id: str) -> None:
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
        chat=chat,
    )
    db.session.add(chat_message)
    db.session.flush()
    logger.info(
        "[PERF_TRACE] API_EXIT create_chat_message chat=%s elapsed_ms=%.2f",
        chat_id, _perf_ms(t0),
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
    db.session.flush()
    logger.info(
        "[PERF_TRACE] API_EXIT update_chat_message message=%s elapsed_ms=%.2f",
        message_id, _perf_ms(t0),
    )
    return chat_message


def delete_message(chat_id: str, message_id: str) -> None:
    db.session.delete(get_message(message_id))
    db.session.flush()
