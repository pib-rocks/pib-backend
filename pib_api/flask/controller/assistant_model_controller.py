from flask import abort, Blueprint

from app import app
from schema.assistant_model_schema import assistant_models_schema, assistant_model_schema
from service import assistant_service
from service.assistant_service import get_assistant_by_id

bp = Blueprint('assistant_controller', __name__)


@bp.route('', methods=['GET'])
def get_all_assistant_models():
    assistant_models = assistant_service.get_all_assistants()
    try:
        assistant_models_dto = assistant_models_schema.dump(assistant_models)
    except:
        abort(500)
    return {"assistantModels": assistant_models_dto}


@bp.route('/<int:assistant_model_id>', methods=['GET'])
def get_assistant_model(assistant_model_id):
    assistant_models = get_assistant_by_id(assistant_model_id)
    try:
        assistant_models_dto = assistant_model_schema.dump(assistant_models)
    except:
        abort(500)
    return assistant_models_dto
