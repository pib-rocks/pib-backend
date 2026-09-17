"""Transition aliases for the controller API used by the existing UI."""

from flask import Blueprint, jsonify, request

from model.controller_model import TINKERFORGE_BRICKLET
from service import controller_service

bp = Blueprint("bricklet_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_bricklets():
    controllers = controller_service.get_all_controllers()
    return jsonify(
        {
            "bricklets": [
                {
                    "brickletNumber": controller.number,
                    "uid": controller.address or "",
                    "type": _legacy_type(controller.number, controller.device_type),
                }
                for controller in controllers
                if controller.kind == TINKERFORGE_BRICKLET
            ]
        }
    )


@bp.route("/<string:bricklet_number>", methods=["GET"])
def get_bricklet(bricklet_number: str):
    controller = controller_service.get_controller(int(bricklet_number))
    return {"uid": controller.address or ""}


@bp.route("/<string:bricklet_number>", methods=["PUT"])
def update_bricklet(bricklet_number: str):
    uid = (request.get_json() or {}).get("uid")
    if not isinstance(uid, str):
        raise ValueError("Bricklet UID must be a string")
    controller = controller_service.set_controller_address(int(bricklet_number), uid)
    return {
        "brickletNumber": controller.number,
        "uid": controller.address or "",
        "type": _legacy_type(controller.number, controller.device_type),
    }


def _legacy_type(number: int, device_type: str | None = None) -> str:
    if device_type is not None:
        return device_type

    # Legacy fallback for controller rows created before device_type was stored.
    if number == 4:
        return "Solid State Relay Bricklet"
    if number in (5, 6, 7):
        return "RGB LED Button Bricklet"
    return "Servo Bricklet"
