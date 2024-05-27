from flask import Blueprint
from controller import motor_controller

blueprint = Blueprint("blueprint", __name__)

blueprint.route("", methods=["GET"])(motor_controller.get_all_motors)
blueprint.route("/<string:name>", methods=["GET"])(motor_controller.get_motor)
blueprint.route("/<string:name>", methods=["PUT"])(motor_controller.update_motor)
blueprint.route("/<string:name>/settings", methods=["GET"])(
    motor_controller.get_motor_settings
)
blueprint.route("/<string:name>/settings", methods=["PUT"])(
    motor_controller.update_motor_settings
)
blueprint.route("/<string:name>/bricklet-pins", methods=["GET"])(
    motor_controller.get_motor_bricklet_pins
)
blueprint.route("/<string:name>/bricklet-pins", methods=["PUT"])(
    motor_controller.update_motor_bricklet_pins
)
