from flask import jsonify, abort, request
from schema.program_schema import programs_schema_without_code, program_schema_name_only, program_schema_without_code
from schema.program_code_schema import program_code_schema
from service import program_service
from app.app import db


def create_program():
    program_dto = program_schema_name_only.load(request.json)
    program = program_service.create_program(program_dto)
    db.session.commit()
    try: return program_schema_without_code.dump(program), 201
    except Exception: abort(500)


def get_all_programs():
    programs = program_service.get_all_programs()
    try: return jsonify({'programs': programs_schema_without_code.dump(programs)})
    except Exception: abort(500)


def get_program(program_number: str):
    program = program_service.get_program(program_number)
    try: return program_schema_without_code.dump(program)
    except Exception: abort(500)


def update_program(program_number: str):
    program_dto = program_schema_name_only.load(request.json)
    program = program_service.update_program(program_number, program_dto)
    db.session.commit()
    try: return program_schema_without_code.dump(program)
    except Exception: abort(500)


def delete_program(program_number: str):
    program_service.delete_program(program_number)
    db.session.commit()
    return '', 204


def get_program_code(program_number: str):
    program = program_service.get_program(program_number)
    try: return program_code_schema.dump({"visual": program.codeVisual})
    except Exception: abort(500)


def update_program_code(program_number: str):
    program_code_dto = program_code_schema.load(request.json)
    program_service.update_program_code(program_number, program_code_dto)
    db.session.commit()
    try: return program_code_schema.dump(program_code_dto)
    except Exception: abort(500)