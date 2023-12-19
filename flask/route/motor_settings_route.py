from flask import Blueprint
from controller import motor_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('/<string:name>', methods=['GET'])(motor_controller.get_motor)
blueprint.route('/', methods=['GET'])(motor_controller.get_motors)
blueprint.route('/', methods=['PUT'])(motor_controller.update_motor)