import sys
from schema.motor_schema import (
    motor_schema,
    motors_schema,
    motor_settings_schema,
    motor_controller_schema,
)
from service import motor_service
from flask import request, jsonify, Blueprint

bp = Blueprint("motor_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_motors():
    motors = motor_service.get_all_motors()
    return jsonify({"motors": motors_schema.dump(motors)})


@bp.route("/<string:name>", methods=["GET"])
def get_motor(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_schema.dump(motor)


@bp.route("/<string:name>", methods=["PUT"])
def update_motor(name: str):
    payload = request.get_json() or {}
    motor_settings_dto = motor_schema.load(payload)
    motor_service.set_motor_controller(name, payload["controller"], payload["channel"])
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    return motor_schema.dump(motor)


@bp.route("/<string:name>/settings", methods=["GET"])
def get_motor_settings(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_settings_schema.dump(motor)


@bp.route("/<string:name>/settings", methods=["PUT"])
def update_motor_settings(name: str):
    motor_settings_dto = motor_settings_schema.load(request.json)
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    return motor_settings_schema.dump(motor)


@bp.route("/<string:name>/controller", methods=["GET"])
def get_motor_controller(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_controller_schema.dump(motor)


@bp.route("/<string:name>/controller", methods=["PUT"])
def update_motor_controller(name: str):
    payload = request.get_json() or {}
    motor = motor_service.set_motor_controller(
        name, payload["controller"], payload["channel"]
    )
    return motor_controller_schema.dump(motor)
