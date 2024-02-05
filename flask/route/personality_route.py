from flask import Blueprint
from controller import personality_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(personality_controller.get_all_personalities)
blueprint.route('', methods=['POST'])(personality_controller.create_personality)
blueprint.route('/<string:personality_id>', methods=['GET'])(personality_controller.get_personality)
blueprint.route('/<string:personality_id>', methods=['PUT'])(personality_controller.update_personality)
blueprint.route('/<string:personality_id>', methods=['DELETE'])(personality_controller.delete_personality)