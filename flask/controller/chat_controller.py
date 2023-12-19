from model.chat_model import Chat
from schema.chat_schema import chat_schema, chats_schema, upload_chat_schema
from app.app import db
from flask import abort, jsonify, request
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