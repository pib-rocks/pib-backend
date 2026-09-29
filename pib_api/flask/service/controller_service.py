from typing import List

from app.app import db
from model.controller_model import TINKERFORGE_BRICKLET, Controller


def get_all_controllers() -> List[Controller]:
    return Controller.query.order_by(Controller.number).all()


def get_controller(controller_number: int) -> Controller:
    return Controller.query.filter(Controller.number == controller_number).one()


def set_controller_address(controller_number: int, address: str) -> Controller:
    controller = get_controller(controller_number)
    controller.address = address
    db.session.flush()
    return controller


def update_controller(controller_number: int, dto: dict) -> Controller:
    controller = get_controller(controller_number)
    if dto.get("kind") not in (None, TINKERFORGE_BRICKLET):
        controller.device_type = None
    for field in ("kind", "device_type", "address", "number", "supply_voltage"):
        if field in dto:
            setattr(controller, field, dto[field])
    db.session.flush()
    return controller
