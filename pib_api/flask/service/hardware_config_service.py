"""Export and import of Bricklet UIDs, pin mappings, and motor limits (PR-1527)."""

from __future__ import annotations

import re
from typing import Any, Dict, List, Set

from app.app import db
from model.bricklet_model import Bricklet
from model.bricklet_pin_model import BrickletPin
from model.motor_model import Motor

SCHEMA_VERSION = 1

# Matches Cerebra's bricklet UID validator (alphanumeric, max 6).
UID_PATTERN = re.compile(r"^[A-Za-z0-9]{1,6}$")

VALID_BRICKLET_TYPES = frozenset(
    {
        "Solid State Relay Bricklet",
        "Servo Bricklet",
        "RGB LED Button Bricklet",
    }
)

MOTOR_SETTING_KEYS = (
    "pulse_width_min",
    "pulse_width_max",
    "rotation_range_min",
    "rotation_range_max",
    "velocity",
    "acceleration",
    "deceleration",
    "period",
    "turned_on",
    "visible",
    "invert",
)


def export_hardware_config() -> Dict[str, Any]:
    """Build a JSON-serializable snapshot of bricklets, pin mappings, and motor limits."""
    bricklets = Bricklet.query.order_by(Bricklet.bricklet_number).all()
    motors = Motor.query.order_by(Motor.name).all()

    return {
        "version": SCHEMA_VERSION,
        "bricklets": [_serialize_bricklet(b) for b in bricklets],
        "motors": [_serialize_motor(m) for m in motors],
    }


def import_hardware_config(payload: Any) -> Dict[str, Any]:
    """Validate and apply a hardware-config document, then return the resulting export."""
    document = validate_hardware_config(payload)
    _apply_hardware_config(document)
    db.session.flush()
    return export_hardware_config()


def validate_hardware_config(payload: Any) -> Dict[str, Any]:
    """Validate schema, UID formats, and duplicate assignments. Raises ValueError on failure."""
    if not isinstance(payload, dict):
        raise ValueError("Hardware config must be a JSON object")

    version = payload.get("version", SCHEMA_VERSION)
    if not isinstance(version, int) or version < 1 or version > SCHEMA_VERSION:
        raise ValueError(f"Unsupported hardware config version: {version!r}")

    if "bricklets" not in payload or not isinstance(payload["bricklets"], list):
        raise ValueError("Hardware config requires a 'bricklets' array")
    if "motors" not in payload or not isinstance(payload["motors"], list):
        raise ValueError("Hardware config requires a 'motors' array")

    bricklets = [_validate_bricklet_entry(entry, index) for index, entry in enumerate(payload["bricklets"])]
    _assert_unique_uids(bricklets)
    _assert_unique_bricklet_numbers(bricklets)

    motors = [_validate_motor_entry(entry, index) for index, entry in enumerate(payload["motors"])]
    _assert_unique_motor_names(motors)
    _assert_pin_bricklets_exist(motors, bricklets)

    return {"version": version, "bricklets": bricklets, "motors": motors}


def _serialize_bricklet(bricklet: Bricklet) -> Dict[str, Any]:
    return {
        "brickletNumber": bricklet.bricklet_number,
        "uid": bricklet.uid or "",
        "type": bricklet.type,
    }


def _serialize_motor(motor: Motor) -> Dict[str, Any]:
    return {
        "name": motor.name,
        "pulseWidthMin": motor.pulse_width_min,
        "pulseWidthMax": motor.pulse_width_max,
        "rotationRangeMin": motor.rotation_range_min,
        "rotationRangeMax": motor.rotation_range_max,
        "velocity": motor.velocity,
        "acceleration": motor.acceleration,
        "deceleration": motor.deceleration,
        "period": motor.period,
        "turnedOn": motor.turned_on,
        "visible": motor.visible,
        "invert": motor.invert,
        "brickletPins": [
            {
                "brickletNumber": pin.bricklet.bricklet_number,
                "pin": pin.pin,
                "invert": pin.invert,
            }
            for pin in motor.bricklet_pins
        ],
    }


def _validate_bricklet_entry(entry: Any, index: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"bricklets[{index}] must be an object")

    bricklet_number = entry.get("brickletNumber", entry.get("bricklet_number"))
    if not isinstance(bricklet_number, int) or isinstance(bricklet_number, bool):
        raise ValueError(f"bricklets[{index}].brickletNumber must be an integer")

    uid = entry.get("uid", "")
    if uid is None:
        uid = ""
    if not isinstance(uid, str):
        raise ValueError(f"bricklets[{index}].uid must be a string")
    uid = uid.strip()
    if uid and not UID_PATTERN.fullmatch(uid):
        raise ValueError(
            f"bricklets[{index}].uid has invalid format '{uid}' "
            "(expected alphanumeric, max 6 characters)"
        )

    bricklet_type = entry.get("type")
    if bricklet_type is not None:
        if not isinstance(bricklet_type, str) or bricklet_type not in VALID_BRICKLET_TYPES:
            raise ValueError(
                f"bricklets[{index}].type '{bricklet_type}' is not a supported Bricklet type"
            )

    return {
        "bricklet_number": bricklet_number,
        "uid": uid or None,
        "type": bricklet_type,
    }


def _validate_motor_entry(entry: Any, index: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"motors[{index}] must be an object")

    name = entry.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"motors[{index}].name must be a non-empty string")

    settings: Dict[str, Any] = {}
    camel_to_snake = {
        "pulseWidthMin": "pulse_width_min",
        "pulseWidthMax": "pulse_width_max",
        "rotationRangeMin": "rotation_range_min",
        "rotationRangeMax": "rotation_range_max",
        "turnedOn": "turned_on",
    }
    for camel, snake in camel_to_snake.items():
        if camel in entry:
            settings[snake] = entry[camel]
        elif snake in entry:
            settings[snake] = entry[snake]

    for key in ("velocity", "acceleration", "deceleration", "period", "visible", "invert"):
        if key in entry:
            settings[key] = entry[key]

    for key, value in settings.items():
        if key in ("turned_on", "visible", "invert"):
            if not isinstance(value, bool):
                raise ValueError(f"motors[{index}].{key} must be a boolean")
        elif not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"motors[{index}].{key} must be an integer")

    pins_raw = entry.get("brickletPins", entry.get("bricklet_pins", []))
    if pins_raw is None:
        pins_raw = []
    if not isinstance(pins_raw, list):
        raise ValueError(f"motors[{index}].brickletPins must be an array")

    pins = [_validate_pin_entry(pin, index, pin_index) for pin_index, pin in enumerate(pins_raw)]
    return {"name": name.strip(), "settings": settings, "bricklet_pins": pins}


def _validate_pin_entry(entry: Any, motor_index: int, pin_index: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"motors[{motor_index}].brickletPins[{pin_index}] must be an object")

    bricklet_number = entry.get("brickletNumber", entry.get("bricklet_number"))
    if not isinstance(bricklet_number, int) or isinstance(bricklet_number, bool):
        raise ValueError(
            f"motors[{motor_index}].brickletPins[{pin_index}].brickletNumber must be an integer"
        )

    pin = entry.get("pin")
    if not isinstance(pin, int) or isinstance(pin, bool):
        raise ValueError(f"motors[{motor_index}].brickletPins[{pin_index}].pin must be an integer")

    invert = entry.get("invert", False)
    if not isinstance(invert, bool):
        raise ValueError(f"motors[{motor_index}].brickletPins[{pin_index}].invert must be a boolean")

    return {"bricklet_number": bricklet_number, "pin": pin, "invert": invert}


def _assert_unique_uids(bricklets: List[Dict[str, Any]]) -> None:
    seen: Set[str] = set()
    for entry in bricklets:
        uid = entry["uid"]
        if not uid:
            continue
        if uid in seen:
            raise ValueError(f"Duplicate Bricklet UID assignment: '{uid}'")
        seen.add(uid)


def _assert_unique_bricklet_numbers(bricklets: List[Dict[str, Any]]) -> None:
    seen: Set[int] = set()
    for entry in bricklets:
        number = entry["bricklet_number"]
        if number in seen:
            raise ValueError(f"Duplicate brickletNumber in import: {number}")
        seen.add(number)


def _assert_unique_motor_names(motors: List[Dict[str, Any]]) -> None:
    seen: Set[str] = set()
    for entry in motors:
        name = entry["name"]
        if name in seen:
            raise ValueError(f"Duplicate motor name in import: '{name}'")
        seen.add(name)


def _assert_pin_bricklets_exist(
    motors: List[Dict[str, Any]], bricklets: List[Dict[str, Any]]
) -> None:
    numbers_in_file = {b["bricklet_number"] for b in bricklets}
    for motor in motors:
        for pin in motor["bricklet_pins"]:
            number = pin["bricklet_number"]
            # Pin may reference a bricklet already in DB even if omitted from this file.
            if number in numbers_in_file:
                continue
            existing = Bricklet.query.filter(Bricklet.bricklet_number == number).first()
            if existing is None:
                raise ValueError(
                    f"Motor '{motor['name']}' references unknown brickletNumber {number}"
                )


def _apply_hardware_config(document: Dict[str, Any]) -> None:
    # Clear UIDs first so swaps cannot hit the unique constraint mid-update.
    target_numbers = [b["bricklet_number"] for b in document["bricklets"]]
    if target_numbers:
        Bricklet.query.filter(Bricklet.bricklet_number.in_(target_numbers)).update(
            {Bricklet.uid: None}, synchronize_session=False
        )
        db.session.flush()

    for entry in document["bricklets"]:
        bricklet = Bricklet.query.filter(
            Bricklet.bricklet_number == entry["bricklet_number"]
        ).one_or_none()
        if bricklet is None:
            if entry["type"] is None:
                raise ValueError(
                    f"Cannot create brickletNumber {entry['bricklet_number']} without a type"
                )
            bricklet = Bricklet(
                bricklet_number=entry["bricklet_number"],
                type=entry["type"],
                uid=entry["uid"],
            )
            db.session.add(bricklet)
        else:
            if entry["type"] is not None and bricklet.type != entry["type"]:
                # Keep existing type for backward compatibility; reject incompatible changes.
                raise ValueError(
                    f"brickletNumber {entry['bricklet_number']} type mismatch: "
                    f"file has '{entry['type']}', database has '{bricklet.type}'"
                )
            bricklet.uid = entry["uid"]
    db.session.flush()

    for entry in document["motors"]:
        motor = Motor.query.filter(Motor.name == entry["name"]).one_or_none()
        if motor is None:
            raise ValueError(f"Unknown motor '{entry['name']}'")

        for key, value in entry["settings"].items():
            setattr(motor, key, value)

        if "bricklet_pins" in entry:
            motor.bricklet_pins.clear()
            db.session.flush()
            for pin_dto in entry["bricklet_pins"]:
                bricklet = Bricklet.query.filter(
                    Bricklet.bricklet_number == pin_dto["bricklet_number"]
                ).one()
                db.session.add(
                    BrickletPin(
                        motor=motor,
                        bricklet=bricklet,
                        pin=pin_dto["pin"],
                        invert=pin_dto["invert"],
                    )
                )
