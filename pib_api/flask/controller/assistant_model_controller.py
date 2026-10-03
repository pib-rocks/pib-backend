from flask import Blueprint, abort

from schema.assistant_model_schema import assistant_model_schema
from schema.provider_schema import provider_schema, providers_schema
from service import assistant_model_service, provider_service

bp = Blueprint("assistant_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_assistant_models():
    """Selection list. Rows without the images capability are not offered."""
    models = provider_service.selectable_models()
    return {"assistantModels": providers_schema.dump(models)}


@bp.route("/<int:assistant_model_id>", methods=["GET"])
def get_assistant_model(assistant_model_id):
    # An existing personality may still reference a row that is not offered
    # for new selection. Lookup by id returns that row, including its flags.
    provider = provider_service.get_model_by_id(assistant_model_id)
    if provider is not None:
        return provider_schema.dump(provider)
    assistant_model = assistant_model_service.get_assistant_model_by_id(
        assistant_model_id
    )
    if assistant_model is None:
        abort(404)
    return assistant_model_schema.dump(assistant_model)
