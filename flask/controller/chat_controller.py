from model.chat_model import Chat
from model.chat_message_model import ChatMessage
from schema.chat_schema import chat_schema, chats_schema, upload_chat_schema
from schema.chat_message_schema import chat_message_post_schema, chat_messages_schema, chat_message_schema
from app.app import db
from flask import abort, jsonify, request
from marshmallow import ValidationError
import uuid

def create_chat():
    error = upload_chat_schema.validate(request.json)
    if error:
        return error, 400
    chat = Chat(request.json.get('topic'), request.json.get('personalityId'))
    chat.chatId = str(uuid.uuid4())
    db.session.add(chat)
    db.session.commit()
    return_chat = Chat.query.filter(Chat.chatId == chat.chatId).first_or_404()
    try:
        return jsonify(chat_schema.dump(return_chat)), 201
    except:
        abort(500)


def get_all_chats():
    all_chats = Chat.query.all()
    try:
        return jsonify({"voiceAssistantChats": chats_schema.dump(all_chats)})
    except:
        abort(500)


def get_chat_by_id(uuid):
    getChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    try:
        return chat_schema.dump(getChat)
    except:
        abort(500)


def update_chat(uuid):
    error = upload_chat_schema.validate(request.json)
    if error:
        return error, 400
    chat = Chat(request.json.get('topic'), request.json.get('personalityId'))
    updateChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    updateChat.topic = chat.topic
    updateChat.personalityId = chat.personalityId
    db.session.add(updateChat)
    db.session.commit()
    updateChat = Chat.query.filter(Chat.chatId == updateChat.chatId).first_or_404()
    try:
        return chat_schema.dump(updateChat)
    except:
        abort(500)


def delete_chat(uuid):
    deleteChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    db.session.delete(deleteChat)
    db.session.commit()
    try:
        return '', 204
    except:
        abort(500)


def create_message(chat_id):
    try:
        data = chat_message_post_schema.load(request.json)
    except ValidationError as error:
        return error.messages, 400
    chat_message = ChatMessage(
        data['isUser'],
        data['content'],
        chat_id
    )
    db.session.add(chat_message)
    db.session.commit()
    try:
        return chat_message_schema.dump(chat_message), 201
    except:
        abort(500)

def get_messages_by_chat_id(chat_id):
    Chat.query.filter(Chat.chatId == chat_id).first_or_404() # TODO: method that only checks if exists?
    chat_messages = ChatMessage.query.filter(ChatMessage.chatId == chat_id).order_by(ChatMessage.timestamp) # TODO proper ordering
    try:
        return jsonify({"messages": chat_messages_schema.dump(chat_messages)}), 200
    except:
        abort(500)