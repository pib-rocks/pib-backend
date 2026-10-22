from flask import Blueprint, jsonify, request
from service import diagnostics_service

bp = Blueprint("diagnostics_controller", __name__)


@bp.route("/summary", methods=["GET"])
def get_diagnostics_summary():
    summary = diagnostics_service.get_summary()
    return jsonify(summary), 200


@bp.route("/bricklets", methods=["GET"])
def get_diagnostics_bricklets():
    bricklets = diagnostics_service.get_bricklets_telemetry()
    return jsonify({"bricklets": bricklets}), 200


@bp.route("/system", methods=["GET"])
def get_diagnostics_system():
    system_info = diagnostics_service.get_system_telemetry()
    return jsonify(system_info), 200


@bp.route("/telemetry", methods=["POST"])
def post_diagnostics_telemetry():
    payload = request.get_json(silent=True)
    if isinstance(payload, list):
        pins = payload
    elif isinstance(payload, dict):
        pins = payload.get("pins") or []
    else:
        pins = []
    updated = diagnostics_service.update_controller_pin_telemetry(pins)
    return jsonify({"updated": updated}), 200
