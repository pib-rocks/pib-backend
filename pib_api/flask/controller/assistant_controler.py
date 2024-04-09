from schema.assistant_schema import assistant_schema
from service import assistant_service
from app.app import db
from flask import jsonify, request, abort

def get_all_assistants_settings():
    assistants = assistant_service.get_all_assistants()
    try: return jsonify({"assistants": assistant_schema.dump(assistants)})
    except Exception: abort(500)
