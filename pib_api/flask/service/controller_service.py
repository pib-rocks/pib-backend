"""Controller rows: the address is what a motor bus is opened with.

A Tinkerforge controller is identified by its Bricklet UID, and that rule stays
the strict one the import and the write path already share. A serial controller
(a Waveshare adapter on a Feetech ST bus, or a CAN interface) carries an
identifier this robot cannot derive: it has to be read from the module and typed
in. So its address is a free-form string, trimmed, up to the column's width -
and an empty value means "not entered yet", which is a legitimate state for a
robot that is being wired.
"""

from typing import List, Optional

from app.app import db
from model.controller_model import TINKERFORGE_BRICKLET, Controller

#: Width of ``Controller.address``; a longer value would be truncated silently.
ADDRESS_MAX_LENGTH = 255

ADDRESS_FIELDS = ("kind", "device_type", "address", "number", "supply_voltage")


def get_all_controllers() -> List[Controller]:
    return Controller.query.order_by(Controller.number).all()


def get_controller(controller_number: int) -> Controller:
    return Controller.query.filter(Controller.number == controller_number).one()


def normalize_address(kind: Optional[str], address: object) -> Optional[str]:
    """The address this controller may store. Raises ValueError with a reason.

    Tinkerforge keeps the Bricklet UID rule, everything else takes a free string:
    an operator reads the identifier off the module and it has no format this
    backend could check. ``None`` or an empty string clears it.
    """
    if address is None:
        return None
    if not isinstance(address, str):
        raise ValueError("address must be a string")
    text = address.strip()
    if not text:
        return None
    if kind == TINKERFORGE_BRICKLET:
        from service import hardware_config_service

        try:
            return hardware_config_service.validate_uid(text, "address") or None
        except ValueError as error:
            raise ValueError(str(error)) from None
    if len(text) > ADDRESS_MAX_LENGTH:
        raise ValueError(
            f"address must be at most {ADDRESS_MAX_LENGTH} characters, "
            f"got {len(text)}"
        )
    return text


def set_controller_address(controller_number: int, address: str) -> Controller:
    """Store an address the caller has already validated.

    The Bricklet route validates the UID before it gets here, so the rule lives in
    one place per route; validating twice would also report the same mistake
    twice. ``update_controller`` is the path that validates (``normalize_address``).
    """
    controller = get_controller(controller_number)
    controller.address = address
    db.session.flush()
    return controller


def update_controller(controller_number: int, dto: dict) -> Controller:
    controller = get_controller(controller_number)
    if dto.get("kind") not in (None, TINKERFORGE_BRICKLET):
        controller.device_type = None
    for field in ADDRESS_FIELDS:
        if field not in dto:
            continue
        if field == "address":
            # The effective kind decides the rule, so a request that changes both
            # is judged on the kind it is asking for.
            effective_kind = dto.get("kind") or controller.kind
            controller.address = normalize_address(effective_kind, dto[field])
            continue
        setattr(controller, field, dto[field])
    db.session.flush()
    return controller
