"""System-level REST endpoints (hardware config export/import, PR-1527)."""

from flask import Blueprint, jsonify, request, Response
import json

from service import hardware_config_service

bp = Blueprint("system_controller", __name__)


@bp.route("/hardware-config/export", methods=["GET"])
def export_hardware_config():
    config = hardware_config_service.export_hardware_config()
    body = json.dumps(config, indent=2)
    response = Response(body, mimetype="application/json")
    response.headers["Content-Disposition"] = (
        "attachment; filename=hardware-config.json"
    )
    return response, 200


@bp.route("/hardware-config/import", methods=["POST"])
def import_hardware_config():
    payload = request.get_json(silent=True)
    if payload is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400
    try:
        result = hardware_config_service.import_hardware_config(payload)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify(result), 200
