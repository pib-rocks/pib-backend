from flask import Blueprint
from controller import bricklet_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(bricklet_controller.get_all_bricklets)
blueprint.route('/<string:bricklet_number>', methods=['GET'])(bricklet_controller.get_bricklet)
blueprint.route('/<string:bricklet_number>', methods=['PUT'])(bricklet_controller.update_bricklet)