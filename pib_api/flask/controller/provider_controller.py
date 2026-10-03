from flask import Blueprint, abort

from schema.provider_schema import provider_accounts_schema, provider_schema
from service import provider_service

bp = Blueprint("provider_controller", __name__)


@bp.route("", methods=["GET"])
def list_providers():
    """Each provider with the models a personality may choose.

    A chat model without the images capability is omitted. A live model is
    its own entry and stays. The filter is the flag, not a list of names.
    The registry is read-only.
    """
    providers = []
    for provider, models in provider_service.providers_with_models():
        provider._listed_models = models
        providers.append(provider)
    return {"providers": provider_accounts_schema.dump(providers)}


@bp.route("/voice-backends", methods=["GET"])
def list_voice_backends():
    """Speech engines a personality may choose.

    Local faster-whisper and Supertone are always present. A provider row
    is present only when its stt or tts capability is set.
    """
    return provider_service.speech_backends()


@bp.route("/default", methods=["GET"])
def get_default_provider():
    return provider_schema.dump(provider_service.get_default_model())


@bp.route("/<int:provider_id>", methods=["GET"])
def get_provider(provider_id: int):
    """One model. The id is the model a personality stores."""
    model = provider_service.get_model_by_id(provider_id)
    if model is None:
        abort(404)
    return provider_schema.dump(model)
