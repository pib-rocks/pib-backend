from app.app import db
from flask import request, Blueprint
from schema.button_program_schema import button_program_schema, button_programs_schema
from model.button_program_model import ButtonProgram
from service import button_program_service
from flask import Blueprint, request
from app.app import db

bp = Blueprint("button_program_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_button_programs():
    button_programs = button_program_service.get_all_button_programs()
    return button_program_schema.dump(button_programs, many=True)


# @bp.route("", methods=["POST"])
# def set_button_program():
#     button_program_dto = button_program_schema.load(request.json)
#     button_program = button_program_service.set_button_program(button_program_dto)
#     db.session.commit()
#     return button_program_schema.dump(button_program)

# @bp.route("/<int:bricklet_id>", methods=["GET"])
# def get_button_program(bricklet_id: int):
#     button_program = button_program_service.get_button_program(bricklet_id)
#     return button_program_schema.dump(button_program)

# @bp.route("/<int:bricklet_id>", methods=["DELETE"])
# def delete_button_program(bricklet_id: int):
#     button_program_service.delete_button_program(bricklet_id)
#     db.session.commit()
#     return "", 204


def update_button_programs():
    button_programs_dto = button_programs_schema.load(request.json)
    updated_button_programs = button_program_service.update_button_programs(
        button_programs_dto
    )
    db.session.commit()
    return button_programs_schema.dump(updated_button_programs)
