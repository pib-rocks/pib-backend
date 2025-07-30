import os
from model.program_model import Program
from pib_api.flask.app.app import app, db
from typing import Any
from pib_blockly import pib_blockly_client


def get_all_programs() -> list[Program]:
    return Program.query.all()


def get_program(program_number: str) -> Program:
    return Program.query.filter(Program.program_number == program_number).one()


def create_program(program_dto: dict[str, Any]) -> Program:
    program = Program(name=program_dto["name"])
    db.session.add(program)
    db.session.flush()
    _create_empty_python_code_file(program.program_number)
    return program


def update_program(program_number: str, program_dto: dict[str, Any]) -> Program:
    program = get_program(program_number)
    program.name = program_dto["name"]
    db.session.flush()
    return program


def delete_program(program_number: str) -> None:
    db.session.delete(get_program(program_number))
    _delete_python_code_file(program_number)
    db.session.flush()


def update_program_code(program_number: str, program_dto: dict[str, Any]) -> None:
    program = get_program(program_number)
    code_visual = program_dto["code_visual"]
    program.code_visual = code_visual
    successful, code_python = pib_blockly_client.code_visual_to_python(code_visual)
    if not successful:
        raise Exception("failed to generate python-code")
    _write_to_python_code_file(program_number, code_python)
    db.session.flush()


def _create_empty_python_code_file(program_number):
    open(_get_code_filepath(program_number), "w").close()


def _write_to_python_code_file(program_number, code_python):
    with open(_get_code_filepath(program_number), "w", encoding="utf-8") as f:
        f.write(code_python)


def _delete_python_code_file(program_number):
    os.remove(_get_code_filepath(program_number))


def _get_code_filepath(program_number):
    return os.path.join(app.config["PYTHON_CODE_DIR"], f"{program_number}.py")
