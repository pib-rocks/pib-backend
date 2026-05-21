from model.button_program_model import ButtonProgram
from service.bricklet_service import get_bricklet
from service.program_service import get_program
from app.app import db


def get_all_button_programs() -> list[ButtonProgram]:
    return ButtonProgram.query.all()


def get_button_program_by_bricklet_number(bricklet_number: int) -> ButtonProgram:
    bricklet = get_bricklet(bricklet_number)
    return ButtonProgram.query.filter_by(bricklet_id=bricklet.id).one()


def update_button_programs(button_programs_dto: list[dict]) -> list[ButtonProgram]:
    for button_program_dto in button_programs_dto:
        bricklet = get_bricklet(button_program_dto.get("brickletNumber"))
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
