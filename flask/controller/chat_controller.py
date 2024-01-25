from service import chat_service
from schema.chat_schema import chat_schema, chats_schema, upload_chat_schema, chat_messages_only_schema
from schema.chat_message_schema import chat_message_post_schema, chat_messages_schema, chat_message_schema
from app.app import db
from flask import abort, jsonify, request


def create_chat():
    chat_dto = upload_chat_schema.load(request.json)
    chat = chat_service.create_chat(chat_dto)
    db.session.commit()
    try: return chat_schema.dump(chat), 201
    except Exception: abort(500)


def get_all_chats():
    chats = chat_service.get_all_chats()
    try: return jsonify({"voiceAssistantChats": chats_schema.dump(chats)})
    except Exception: abort(500)


def get_chat_by_id(chat_id: str):
    chat = chat_service.get_chat(chat_id)
    try: return chat_schema.dump(chat)
    except Exception: abort(500)


def update_chat(chat_id: str):
    chat_dto = upload_chat_schema.load(request.json)
    chat = chat_service.update_chat(chat_id, chat_dto)
    db.session.commit()
    try: return chat_schema.dump(chat)
    except Exception: abort(500)


def delete_chat(chat_id: str):
    chat_service.delete_chat(chat_id)
    db.session.commit()
    return '', 204


def create_message(chat_id: str):
    chat_message_dto = chat_message_post_schema.load(request.json)
    chat_message = chat_service.create_chat_message(chat_id, chat_message_dto)
    db.session.commit()
    try: return chat_message_schema.dump(chat_message), 201
    except Exception: abort(500)
    

def get_messages_by_chat_id(chat_id: str):
    chat = chat_service.get_chat(chat_id)
    try: return chat_messages_only_schema.dump(chat)
    except Exception: abort(500)


def delete_message(chat_id: str, message_id: str):
    chat_service.delete_message(chat_id, message_id)
    db.session.commit()
    return '', 204