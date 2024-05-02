from flask import abort, Blueprint

from schema.assistant_model_schema import assistant_models_schema, assistant_model_schema
from service import assistant_model_service

bp = Blueprint('assistant_controller', __name__)


@bp.route('', methods=['GET'])
def get_all_assistant_models():
    assistant_models = assistant_model_service.get_all_assistant_models()
    try:
        assistant_models_dto = assistant_models_schema.dump(assistant_models)
    except:
        abort(500)
    return {"assistantModels": assistant_models_dto}


@bp.route('/<int:assistant_model_id>', methods=['GET'])
def get_assistant_model(assistant_model_id):
    assistant_models = assistant_model_service.get_assistant_model_by_id(assistant_model_id)
    try:
        assistant_models_dto = assistant_model_schema.dump(assistant_models)
    except:
        abort(500)
    return assistant_models_dto
