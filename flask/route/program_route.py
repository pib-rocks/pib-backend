from flask import Blueprint
from controller import program_controller

blueprint = Blueprint('blueprint', __name__)

blueprint.route('/', methods=['POST'])(program_controller.create_program)
blueprint.route('/', methods=['GET'])(program_controller.get_all_programs)
blueprint.route('/<string:program_number>', methods=['GET'])(program_controller.get_program_by_number)
blueprint.route('/<string:program_number>', methods=['PUT'])(program_controller.update_program_by_number)
blueprint.route('/<string:program_number>', methods=['DELETE'])(program_controller.delete_program_by_number)
blueprint.route('/<string:program_number>/code', methods=['GET'])(program_controller.get_program_code_by_number)
blueprint.route('/<string:program_number>/code', methods=['PUT'])(program_controller.update_program_code_by_number)
