from flask import Blueprint, jsonify, request

from schema.controller_schema import controller_schema, controllers_schema
from service import controller_service

bp = Blueprint("controller_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_controllers():
    controllers = controller_service.get_all_controllers()
    return jsonify({"controllers": controllers_schema.dump(controllers)})


@bp.route("/<int:controller_number>", methods=["GET"])
def get_controller(controller_number: int):
    return controller_schema.dump(controller_service.get_controller(controller_number))


@bp.route("/<int:controller_number>", methods=["PUT"])
def update_controller(controller_number: int):
    payload = controller_schema.load(request.get_json() or {}, partial=True)
    try:
        controller = controller_service.update_controller(controller_number, payload)
    except ValueError as error:
        # A refused identifier is the caller's mistake, not a server fault.
        return jsonify({"error": str(error)}), 400
    return controller_schema.dump(controller)
