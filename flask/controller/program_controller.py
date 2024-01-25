from model.program_model import Program
from flask import jsonify, abort, request
from schema.program_schema import programs_schema_without_program, program_schema_name_only, program_schema_without_program
from schema.program_code_schema import program_code_schema, program_code_visual_only_schema
from service import program_service
from marshmallow import ValidationError
from app.app import db


def create_program():
    error = program_schema_name_only.validate(request.json)
    if error:
        return error, 400
    created = Program(request.json.get("name"))
    db.session.add(created)
    db.session.commit()
    program_service.create_empty_python_code_file(created.programNumber)
    try:
        return program_schema_without_program.dump(created)
    except:
        abort(500)


def get_all_programs():
    all_programs = Program.query.all()
    try:
        return jsonify({"programs": programs_schema_without_program.dump(all_programs)})
    except:
        abort(500)


def get_program_by_number(program_number):
    program = Program.query.filter(Program.programNumber == program_number).first_or_404()
    try:
        return program_schema_without_program.dump(program)
    except:
        abort(500)


def update_program_by_number(program_number):
    try:
        data = program_schema_name_only.load(request.json)
    except ValidationError as error:
        return error.messages, 400
    program = Program.query.filter(Program.programNumber == program_number).first_or_404()
    program.name = data["name"]
    db.session.add(program)
    db.session.commit()
    try:
        return program_schema_without_program.dump(program)
    except:
        abort(500)


def delete_program_by_number(program_number):
    program_service.delete_python_code_file(program_number)
    delete_program = Program.query.filter(Program.programNumber == program_number).first_or_404()
    try:
        db.session.delete(delete_program)
        db.session.commit()
        return '', 204
    except:
        abort(500)


def get_program_code_by_number(program_number):
    program = Program.query.filter(Program.programNumber == program_number).first_or_404()
    return program_code_visual_only_schema.dump({
        "visual": program.codeVisual
    })


def update_program_code_by_number(program_number):
    try:
        data = program_code_schema.load(request.json)
    except ValidationError as error:
        return error.messages, 400
    program = Program.query.filter(Program.programNumber == program_number).first_or_404()
    program.codeVisual = data["visual"]
    db.session.commit()
    program_service.write_to_python_code_file(program_number, data["python"])
    return program_code_visual_only_schema.dump(data)