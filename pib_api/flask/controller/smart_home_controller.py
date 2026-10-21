"""SmartHome REST endpoints (Home Assistant hub and Matter commissioning)."""

from flask import Blueprint, jsonify, request
from sqlalchemy.exc import NoResultFound

from service import smart_home_service
from service.smart_home_service import SmartHomeValidationError

bp = Blueprint("smart_home_controller", __name__)


@bp.route("/status", methods=["GET"])
def get_status():
    return jsonify(smart_home_service.get_hub_status()), 200


@bp.route("/initialize", methods=["POST"])
def initialize_hub():
    return jsonify(smart_home_service.initialize_hub()), 200


@bp.route("/devices", methods=["GET"])
def list_devices():
    room = request.args.get("room")
    domain = request.args.get("domain")
    voice_only = request.args.get("voiceOnly", "").lower() in {"1", "true", "yes"}
    devices = smart_home_service.list_devices(
        room=room, domain=domain, voice_only=voice_only
    )
    return jsonify({"devices": devices}), 200


@bp.route("/devices", methods=["POST"])
def create_device():
    payload = request.get_json(silent=True) or {}
    entity_id = payload.get("entityId")
    if not entity_id:
        raise SmartHomeValidationError("entityId is required.")
    device = smart_home_service.register_or_update_device(entity_id, payload)
    return jsonify(device), 201


@bp.route("/devices/<path:entity_id>", methods=["PATCH", "PUT"])
def update_device(entity_id: str):
    payload = request.get_json(silent=True) or {}
    device = smart_home_service.register_or_update_device(entity_id, payload)
    return jsonify(device), 200


@bp.route("/devices/<path:entity_id>", methods=["DELETE"])
def delete_device(entity_id: str):
    try:
        smart_home_service.delete_device(entity_id)
    except NoResultFound:
        raise
    return jsonify({"deleted": True, "entityId": entity_id}), 200


@bp.route("/devices/<path:entity_id>/action", methods=["POST"])
def device_action(entity_id: str):
    payload = request.get_json(silent=True) or {}
    action = payload.get("action")
    if not action:
        raise SmartHomeValidationError("action is required.")
    result = smart_home_service.execute_device_action(
        entity_id,
        action,
        value=payload.get("value"),
        extra=payload.get("extra") if isinstance(payload.get("extra"), dict) else None,
    )
    return jsonify(result), 200


@bp.route("/matter/commission", methods=["POST"])
def commission_matter():
    payload = request.get_json(silent=True) or {}
    pairing_code = payload.get("pairingCode")
    if not pairing_code:
        raise SmartHomeValidationError("pairingCode is required.")
    result = smart_home_service.commission_matter_device(
        pairing_code,
        room=payload.get("room") or "General",
        friendly_name=payload.get("friendlyName"),
        domain=payload.get("domain") or "light",
        wifi_ssid=payload.get("wifiSsid"),
        wifi_password=payload.get("wifiPassword"),
    )
    return jsonify(result), 201
