from flask import Blueprint
from controller import camera_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(camera_controller.get_camera_settings)
blueprint.route('', methods=['PUT'])(camera_controller.update_camera_settings)