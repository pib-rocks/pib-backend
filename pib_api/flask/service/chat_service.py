from typing import Any, List

from app.app import db
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from service import personality_service


def get_all_chats() -> List[Chat]:
    return Chat.query.all()


def get_chat(chat_id: str) -> Chat:
    return Chat.query.filter(Chat.chat_id == chat_id).one()

def get_message(chat_id: str, message_id: str) -> ChatMessage:
    return ChatMessage.query.filter(
        (ChatMessage.message_id == message_id) & (ChatMessage.chat_id == chat_id)
    ).one()

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


def create_chat_message(chat_id: str, chat_message_dto: Any) -> ChatMessage:
    chat = get_chat(chat_id)
    chat_message = ChatMessage(
        is_user=chat_message_dto["is_user"],
        content=chat_message_dto["content"],
        chat=chat,
    )
    db.session.add(chat_message)
    db.session.flush()
    return chat_message

def update_chat_message(chat_message_dto: dict[str, Any], chat_id: str, message_id: str) ->ChatMessage:
    chat_message = get_message(chat_id, message_id)
    chat_message.content = chat_message_dto['content']
    db.session.flush()
    return chat_message


def delete_message(chat_id: str, message_id: str) -> None:
    db.session.delete(get_message(chat_id, message_id))
    db.session.flush()
