"""Legacy compatibility routes; live microphone I/O is owned by ros-audio-io."""

from flask import Blueprint, jsonify, request
from service import microphone_array_service

bp = Blueprint("microphone_array_controller", __name__)


@bp.route("/health", methods=["GET"])
def get_health():
    health = microphone_array_service.health()
    return jsonify(health), 200


@bp.route("/telemetry", methods=["GET"])
def get_telemetry():
    telemetry = microphone_array_service.get_telemetry()
    return jsonify(telemetry), 200


@bp.route("/tuning", methods=["GET"])
def get_tuning():
    tuning = microphone_array_service.get_tuning()
    return jsonify(tuning), 200


@bp.route("/desired-state", methods=["GET"])
def get_desired_state():
    desired_state = microphone_array_service.get_desired_state()
    return jsonify(desired_state), 200


@bp.route("/tuning", methods=["POST"])
def update_tuning():
    """Validate a legacy request without applying it to the USB device."""

    payload = request.get_json(silent=True)
    if payload is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400
    try:
        tuning = microphone_array_service.update_tuning(payload)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception as exc:  # pragma: no cover - unexpected hardware failures
        return jsonify({"error": str(exc)}), 500
    return jsonify(tuning), 200
