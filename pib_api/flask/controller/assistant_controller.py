from flask import abort, Blueprint, make_response

from app import app
from schema.assistant_schema import assistants_schema, assistant_schema
from service import assistant_service
from service.assistant_service import get_assistant_by_id

bp = Blueprint('assistant_controller', __name__)


@bp.route('', methods=['GET'])
def get_all_assistants_settings():
    try:
        assistants = assistant_service.get_all_assistants()
        assistant_dtos = assistants_schema.dump(assistants)
    except Exception as e:
        app.logger.error(e)
        abort(500, description="Internal Server Error")

    return make_response({"voiceAssistantModels": assistant_dtos}, 200)


@bp.route('/<int:assistant_id>', methods=['GET'])
def get_assistant(assistant_id):
    try:
        assistant = get_assistant_by_id(assistant_id)
        assistant_dto = assistant_schema.dump(assistant)
    except Exception as e:
        app.logger.error(e)
        abort(500, description="Internal Server Error")

    return make_response(assistant_dto, 200)
