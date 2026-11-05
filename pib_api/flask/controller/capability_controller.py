from flask import Blueprint, jsonify

from pib_capability.manifest import load_manifest

bp = Blueprint("capability_controller", __name__)


@bp.route("", methods=["GET"])
def get_capabilities():
    return jsonify(load_manifest()), 200
