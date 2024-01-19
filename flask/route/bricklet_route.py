from flask import Blueprint
from controller import bricklet_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(bricklet_controller.get_all_bricklets)