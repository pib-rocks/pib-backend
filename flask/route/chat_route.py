from flask import Blueprint
from controller import chat_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('/', methods=['POST'])(chat_controller.create_chat)
blueprint.route('/', methods=['GET'])(chat_controller.get_all_chats)
blueprint.route('/<string:uuid>', methods=['GET'])(chat_controller.get_chat_by_id)
blueprint.route('/<string:uuid>', methods=['PUT'])(chat_controller.update_chat)
blueprint.route('/<string:uuid>', methods=['DELETE'])(chat_controller.delete_chat)