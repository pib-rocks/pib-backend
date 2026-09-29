from flask import Blueprint, abort

from schema.provider_schema import provider_schema, providers_schema
from service import provider_service

bp = Blueprint("provider_controller", __name__)


@bp.route("", methods=["GET"])
def list_providers():
    """Registry rows offered for selection.

    A row without the images capability is omitted. The filter is the flag,
    not a list of provider names. The registry is read-only.
    """
    return {"providers": providers_schema.dump(provider_service.selectable_providers())}


@bp.route("/default", methods=["GET"])
def get_default_provider():
    return provider_schema.dump(provider_service.get_default_provider())


@bp.route("/<int:provider_id>", methods=["GET"])
def get_provider(provider_id: int):
    provider = provider_service.get_provider_by_id(provider_id)
    if provider is None:
        abort(404)
    return provider_schema.dump(provider)
