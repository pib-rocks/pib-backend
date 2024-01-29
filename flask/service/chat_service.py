from typing import Any
from model.chat_model import Chat
from model.chat_message_model import ChatMessage
from service import personality_service
from app.app import db
import uuid


def get_all_chats() -> list[Chat]:
    return Chat.query.all()


def get_chat(chat_id: str) -> Chat:
    return Chat.query.filter(Chat.chatId == chat_id).one()


def create_chat(chat_dto: dict[str, Any]) -> Chat:
    personality = personality_service.get_personality(chat_dto['personalityId'])
    chat = Chat(
        topic=chat_dto['topic'], 
        chatId=str(uuid.uuid4()), 
        personality=personality)
    db.session.add(chat)
    db.session.flush()
    return chat


def update_chat(chat_id: str, chat_dto: dict[str, Any]) -> Chat:
    chat = get_chat(chat_id)
    chat.topic = chat_dto['topic']
    db.session.flush()
    return chat


def delete_chat(chat_id: str) -> None:
    db.session.delete(get_chat(chat_id))
    db.session.flush()


def create_chat_message(chat_id: str, chat_message_dto: dict[str, Any]) -> ChatMessage:
    chat = get_chat(chat_id)
    chat_message = ChatMessage(
        messageId=str(uuid.uuid4()),
        isUser=chat_message_dto['isUser'],
        content=chat_message_dto['content'],
        chat=chat)
    db.session.add(chat_message)
    db.session.flush()
    return chat_message


def delete_message(chat_id: str, message_id: str) -> None:
    message = ChatMessage.query.filter(
        (ChatMessage.messageId == message_id) &
        (ChatMessage.chatId == chat_id)).one()
    db.session.delete(message)
    db.session.flush()