from flask import Blueprint, jsonify
from service.version_service import read_app_version

bp = Blueprint("version_controller", __name__)


@bp.route("", methods=["GET"])
def get_version():
    return jsonify({"version": read_app_version()}), 200
