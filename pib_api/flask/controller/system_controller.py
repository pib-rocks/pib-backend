"""System-level REST endpoints."""

import json

from flask import Blueprint, jsonify, request, Response
from sqlalchemy import func

from model.controller_model import (
    Controller,
    FEETECH_ST_SERIAL,
    ROBSTRIDE_CAN,
    TINKERFORGE_BRICKLET,
)
from seed_profiles import PROFILES, resolve_variant_and_source
from service import hardware_config_service, revision_service, update_service
from service.system_property_service import (
    ALLOWED_HARDWARE_VARIANTS,
    HARDWARE_VARIANT_KEY,
    PROPERTY_REGISTRY,
    get_property,
    get_variant,
    list_properties,
)

bp = Blueprint("system_controller", __name__)

# This describes the API's observable/configurable surface. The motor node's
# actuator registry remains authoritative for actually driving each kind.
COMMON_SETTINGS = [
    "pulse_width",
    "rotation_range",
    "velocity",
    "acceleration",
    "period",
]
CONTROLLER_CAPABILITIES = {
    TINKERFORGE_BRICKLET: {
        "feedback": ["current", "target_position"],
        "meaningfulSettings": COMMON_SETTINGS,
    },
    FEETECH_ST_SERIAL: {
        "feedback": [
            "current",
            "target_position",
            "actual_position",
            "temperature",
        ],
        "meaningfulSettings": COMMON_SETTINGS + ["current_limit", "torque_limit"],
    },
    ROBSTRIDE_CAN: {
        "feedback": [
            "current",
            "target_position",
            "actual_position",
            "temperature",
        ],
        "meaningfulSettings": COMMON_SETTINGS + ["current_limit", "torque_limit"],
    },
}


@bp.route("/hardware-variant", methods=["GET"])
def get_hardware_variant():
    stored = get_property(HARDWARE_VARIANT_KEY)
    if stored is not None:
        variant = get_variant()
        source = (
            stored.source if stored.value in ALLOWED_HARDWARE_VARIANTS else "fallback"
        )
    else:
        resolved_variant, resolved_source = resolve_variant_and_source()
        if resolved_variant in ALLOWED_HARDWARE_VARIANTS:
            variant = resolved_variant
            source = resolved_source
        else:
            variant = get_variant()
            source = "fallback"

    return (
        jsonify(
            {
                "variant": variant,
                "source": source,
                "supported": list(ALLOWED_HARDWARE_VARIANTS),
                "implementedVariants": sorted(PROFILES),
                "seedProfileImplemented": variant in PROFILES,
            }
        ),
        200,
    )


@bp.route("/hardware-capabilities", methods=["GET"])
def get_hardware_capabilities():
    installed_counts = {
        kind: count
        for kind, count in Controller.query.with_entities(
            Controller.kind, func.count(Controller.id)
        )
        .group_by(Controller.kind)
        .all()
    }
    capabilities = [
        {
            "kind": kind,
            "installedControllers": installed_counts.get(kind, 0),
            **declaration,
        }
        for kind, declaration in CONTROLLER_CAPABILITIES.items()
    ]
    return jsonify({"capabilities": capabilities}), 200


@bp.route("/properties", methods=["GET"])
def get_system_properties():
    properties = [
        {
            "key": item.key,
            "value": item.value,
            "valueType": item.value_type,
            "source": item.source,
            "updatedAt": item.updated_at.isoformat(),
        }
        for item in list_properties()
        if item.key in PROPERTY_REGISTRY
    ]
    return jsonify({"properties": properties}), 200


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


@bp.route("/update", methods=["POST"])
def start_update():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Request body must be a JSON object"}), 400

    running_signal = update_service.program_running_signal()
    if running_signal is True:
        return jsonify({"error": "An installed program is currently running"}), 409

    try:
        update_request = update_service.build_request(
            channel=payload.get("channel", "release"),
            force=payload.get("force", False),
            confirmation=payload.get("confirmation"),
            actor=request.remote_addr or "unknown",
        )
        status = update_service.enqueue_update(update_request)
    except update_service.UpdateValidationError as error:
        return jsonify({"error": str(error)}), 400
    except update_service.UpdateNotInstalledError as error:
        return jsonify({"error": str(error), "state": error.state}), 503
    except update_service.UpdateConflictError as error:
        return jsonify({"error": str(error), "status": error.status}), 409

    return (
        jsonify(
            {
                "job": update_request,
                "status": status,
                "programRunningSignal": (
                    "available" if running_signal is not None else "unavailable"
                ),
            }
        ),
        202,
    )


@bp.route("/update/status", methods=["GET"])
def get_update_status():
    status = update_service.get_status()
    code = 503 if status["state"] in {"not_installed", "runner_missing"} else 200
    return jsonify(status), code


@bp.route("/update/check", methods=["POST"])
def check_update_available():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "Request body must be a JSON object"}), 400
    try:
        check_request = update_service.build_check_request(
            channel=payload.get("channel", "release"),
            actor=request.remote_addr or "unknown",
        )
        update_service.enqueue_check(check_request)
    except update_service.UpdateValidationError as error:
        return jsonify({"error": str(error)}), 400
    except update_service.UpdateNotInstalledError as error:
        return jsonify({"error": str(error), "state": error.state}), 503
    except update_service.UpdateConflictError as error:
        return jsonify({"error": str(error), "status": error.status}), 409
    return jsonify(check_request), 202


@bp.route("/update/available", methods=["GET"])
def get_update_available():
    try:
        available = update_service.get_available()
    except update_service.UpdateNotInstalledError as error:
        return jsonify({"error": str(error), "state": error.state}), 503
    return jsonify(available), 200


@bp.route("/update/log", methods=["GET"])
def get_update_log():
    try:
        offset = int(request.args.get("offset", "0"))
        result = update_service.read_log(offset)
    except (TypeError, ValueError, update_service.UpdateValidationError) as error:
        return jsonify({"error": str(error)}), 400
    except update_service.UpdateNotInstalledError as error:
        return jsonify({"error": str(error), "state": "not_installed"}), 503
    return jsonify(result), 200


@bp.route("/update/cancel", methods=["POST"])
def cancel_update():
    try:
        status = update_service.request_cancel()
    except update_service.UpdateNotInstalledError as error:
        return jsonify({"error": str(error), "state": error.state}), 503
    except update_service.UpdateConflictError as error:
        return jsonify({"error": str(error), "status": error.status}), 409
    return jsonify({"status": status}), 202


@bp.route("/revision", methods=["GET"])
def get_revision():
    return jsonify(revision_service.installed_revisions()), 200
