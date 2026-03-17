from model.button_program_model import ButtonProgram
from service.bricklet_service import get_bricklet
from service.program_service import get_program
from app.app import db


def get_all_button_programs() -> list[ButtonProgram]:
    return ButtonProgram.query.all()


# def set_button_program(button_program_dto: dict) -> ButtonProgram:
#     button_program = ButtonProgram(
#         bricklet_id=button_program_dto["brickletId"],
#         program_id=button_program_dto["programId"],
#     )
#     db.session.add(button_program)
#     db.session.flush()
#     return button_program


def get_button_program_by_bricklet_number(bricklet_number: int) -> ButtonProgram:
    bricklet = get_bricklet(bricklet_number)
    return ButtonProgram.query.filter_by(bricklet_id=bricklet.id).one()


# def delete_button_program(bricklet_number: int) -> None:
#     button_program = get_button_program_by_bricklet_number(bricklet_number)
#     db.session.delete(button_program)
#     db.session.flush()


def update_button_program(button_programs_dto: list[dict]) -> ButtonProgram:
    for button_program_dto in button_programs_dto:
        bricklet = get_bricklet(button_program_dto["brickletNumber"])
        program = (
            get_program(button_program_dto["programNumber"])
            if button_program_dto["programNumber"]
            else None
        )
        button_program = get_button_program_by_bricklet_number(
            button_program_dto["brickletNumber"]
        )
        button_program.program_id = program.id if program else None
    db.session.flush()
    return get_all_button_programs()
