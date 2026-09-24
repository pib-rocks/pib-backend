from model.button_program_model import ButtonProgram
from service.controller_service import get_controller
from service.program_service import get_program
from app.app import db
from werkzeug.exceptions import UnprocessableEntity


def get_all_button_programs() -> list[ButtonProgram]:
    return ButtonProgram.query.all()


def get_button_program_by_bricklet_number(bricklet_number: int) -> ButtonProgram:
    controller = get_controller(bricklet_number)
    button_program = ButtonProgram.query.filter_by(
        controller_id=controller.id
    ).one_or_none()
    if button_program is None:
        raise UnprocessableEntity(
            "No button program exists for "
            f"buttonProgramUpdates[].brickletNumber={bricklet_number}."
        )
    return button_program


def update_button_programs(button_programs_dto: list[dict]) -> list[ButtonProgram]:
    for button_program_dto in button_programs_dto:
        program = (
            get_program(button_program_dto.get("programNumber"))
            if button_program_dto.get("programNumber")
            else None
        )
        button_program = get_button_program_by_bricklet_number(
            button_program_dto["brickletNumber"]
        )
        button_program.program_id = program.id if program else None
    db.session.flush()
    return get_all_button_programs()
