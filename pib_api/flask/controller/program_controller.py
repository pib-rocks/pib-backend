from flask import jsonify, request, Blueprint
from schema.program_schema import (
    program_schema_name_only,
    program_schema_without_code,
    programs_schema_without_code,
    program_schema_code_visual_only,
)
from service import program_service


bp = Blueprint("program_controller", __name__)


@bp.route("", methods=["POST"])
def create_program():
    program_dto = program_schema_name_only.load(request.json)
    program = program_service.create_program(program_dto)
    return program_schema_without_code.dump(program), 201


@bp.route("", methods=["GET"])
def get_all_programs():
    programs = program_service.get_all_programs()
    return jsonify({"programs": programs_schema_without_code.dump(programs)})


@bp.route("/<string:program_number>", methods=["GET"])
def get_program(program_number: str):
    program = program_service.get_program(program_number)
    return program_schema_without_code.dump(program)


@bp.route("/<string:program_number>", methods=["PUT"])
def update_program(program_number: str):
    program_dto = program_schema_name_only.load(request.json)
    program = program_service.update_program(program_number, program_dto)
    return program_schema_without_code.dump(program)


@bp.route("/<string:program_number>", methods=["DELETE"])
def delete_program(program_number: str):
    program_service.delete_program(program_number)
    return "", 204


@bp.route("/<string:program_number>/code", methods=["GET"])
def get_program_code(program_number: str):
    program = program_service.get_program(program_number)
    return program_schema_code_visual_only.dump(program)


@bp.route("/<string:program_number>/code", methods=["PUT"])
def update_program_code(program_number: str):
    program_dto = program_schema_code_visual_only.load(request.json)
    program_service.update_program_code(program_number, program_dto)
    return program_schema_code_visual_only.dump(program_dto)
