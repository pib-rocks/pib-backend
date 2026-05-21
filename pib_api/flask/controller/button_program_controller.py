from app.app import db
from flask import request, Blueprint
from schema.button_program_schema import (
    button_programs_schema,
    button_programs_load_schema,
)
from model.button_program_model import ButtonProgram
from service import button_program_service

bp = Blueprint("button_program_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_button_programs():
    button_programs = button_program_service.get_all_button_programs()
    return {"buttonPrograms": button_programs_schema.dump(button_programs)}


@bp.route("", methods=["PUT"])
def update_button_programs():
    button_programs_dto = button_programs_load_schema.load(
        request.json.get("buttonProgramUpdates")
    )
    updated_button_programs = button_program_service.update_button_programs(
        button_programs_dto
    )
    db.session.commit()
    return {"buttonPrograms": button_programs_schema.dump(updated_button_programs)}, 200
