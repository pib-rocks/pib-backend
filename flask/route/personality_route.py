from flask import Blueprint
from controller import personality_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(personality_controller.get_all_personalities)
blueprint.route('/<string:uuid>', methods=['GET'])(personality_controller.get_personality_by_id)
blueprint.route('', methods=['POST'])(personality_controller.create_personality)
blueprint.route('/<string:uuid>', methods=['PUT'])(personality_controller.update_personality)
blueprint.route('/<string:uuid>', methods=['DELETE'])(personality_controller.delete_personality)