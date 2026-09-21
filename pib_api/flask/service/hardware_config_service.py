"""Export and import of controllers, motor mappings, and motor limits."""

from __future__ import annotations

import re
from typing import Any, Dict, List, Set

from app.app import db
from model.controller_model import (
    SUPPORTED_CONTROLLER_KINDS,
    TINKERFORGE_DEVICE_TYPES,
    TINKERFORGE_BRICKLET,
    Controller,
)
from model.motor_model import Motor

SCHEMA_VERSION = 2

# Tinkerforge UIDs are Base58, so '0', 'O', 'I' and 'l' are not UID characters.
UID_PATTERN = re.compile(r"^[1-9A-HJ-NP-Za-km-z]{1,6}$")
UID_FORBIDDEN_CHARACTERS = "0OIl"
UID_RULE_DESCRIPTION = (
    "expected Base58, max 6 characters; "
    "'0', 'O', 'I' and 'l' are not valid UID characters"
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
    "current_limit",
    "torque_limit",
)


def validate_uid(value: Any, field: str = "uid") -> str:
    """Return the stripped Bricklet UID, or raise ValueError describing why not.

    This is the single UID rule for the whole backend: the import validator and
    the write path both go through it, so the two layers cannot drift apart.
    The empty string means "not configured" and stays allowed - the motors node
    skips such a controller.
    """
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    address = value.strip()
    if not address:
        return ""
    if not UID_PATTERN.fullmatch(address):
        raise ValueError(
            f"{field} has invalid format '{address}' ({UID_RULE_DESCRIPTION})"
        )
    return address


def export_hardware_config(variant: str | None = None) -> Dict[str, Any]:
    controllers = Controller.query.order_by(Controller.number).all()
    motors = Motor.query.order_by(Motor.name).all()
    result = {
        "version": SCHEMA_VERSION,
        "controllers": [_serialize_controller(c) for c in controllers],
        "motors": [_serialize_motor(m) for m in motors],
    }
    if variant is not None:
        result["variant"] = variant
    return result


def import_hardware_config(payload: Any) -> Dict[str, Any]:
    document = validate_hardware_config(payload)
    _apply_hardware_config(document)
    db.session.flush()
    return export_hardware_config(document.get("variant"))


def validate_hardware_config(payload: Any) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("Hardware config must be a JSON object")

    version = payload.get("version", 1)
    if (
        not isinstance(version, int)
        or isinstance(version, bool)
        or version not in (1, 2)
    ):
        raise ValueError(f"Unsupported hardware config version: {version!r}")

    variant = payload.get("variant")
    if variant is not None and (not isinstance(variant, str) or not variant.strip()):
        raise ValueError("Hardware config variant must be a non-empty string")

    if "motors" not in payload or not isinstance(payload["motors"], list):
        raise ValueError("Hardware config requires a 'motors' array")

    if version == 1:
        document = _validate_v1_document(payload)
    else:
        document = _validate_v2_document(payload)
    if variant is not None:
        document["variant"] = variant.strip()
    return document


def _validate_v1_document(payload: Dict[str, Any]) -> Dict[str, Any]:
    if "bricklets" not in payload or not isinstance(payload["bricklets"], list):
        raise ValueError("Hardware config version 1 requires a 'bricklets' array")
    controllers = [
        _validate_v1_bricklet(entry, index)
        for index, entry in enumerate(payload["bricklets"])
    ]
    motors = [
        _validate_motor_entry(entry, index, version=1)
        for index, entry in enumerate(payload["motors"])
    ]
    _validate_document_references(controllers, motors)
    return {"version": 1, "controllers": controllers, "motors": motors}


def _validate_v2_document(payload: Dict[str, Any]) -> Dict[str, Any]:
    if "controllers" not in payload or not isinstance(payload["controllers"], list):
        raise ValueError("Hardware config version 2 requires a 'controllers' array")
    controllers = [
        _validate_controller_entry(entry, index)
        for index, entry in enumerate(payload["controllers"])
    ]
    motors = [
        _validate_motor_entry(entry, index, version=2)
        for index, entry in enumerate(payload["motors"])
    ]
    _validate_document_references(controllers, motors)
    return {"version": 2, "controllers": controllers, "motors": motors}


def _serialize_controller(controller: Controller) -> Dict[str, Any]:
    return {
        "kind": controller.kind,
        "deviceType": controller.device_type,
        "address": controller.address or "",
        "number": controller.number,
        "supplyVoltage": controller.supply_voltage,
    }


def _serialize_motor(motor: Motor) -> Dict[str, Any]:
    result = {
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
        "controllerNumber": motor.controller.number if motor.controller else None,
        "channel": motor.channel,
    }
    if motor.current_limit is not None:
        result["currentLimit"] = motor.current_limit
    if motor.torque_limit is not None:
        result["torqueLimit"] = motor.torque_limit
    return result


def _validate_v1_bricklet(entry: Any, index: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"bricklets[{index}] must be an object")
    number = entry.get("brickletNumber", entry.get("bricklet_number"))
    _require_integer(number, f"bricklets[{index}].brickletNumber")
    address = entry.get("uid", "")
    if address is None:
        address = ""
    address = validate_uid(address, f"bricklets[{index}].uid")
    device_type = entry.get("deviceType", entry.get("type"))
    _validate_device_type(
        device_type,
        TINKERFORGE_BRICKLET,
        f"bricklets[{index}].deviceType",
    )
    return {
        "kind": TINKERFORGE_BRICKLET,
        "device_type": device_type,
        "address": address or None,
        "number": number,
        "supply_voltage": None,
        "from_v1": True,
    }


def _validate_controller_entry(entry: Any, index: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"controllers[{index}] must be an object")
    kind = entry.get("kind")
    if kind not in SUPPORTED_CONTROLLER_KINDS:
        raise ValueError(f"controllers[{index}].kind {kind!r} is not supported")
    device_type = entry.get("deviceType")
    _validate_device_type(device_type, kind, f"controllers[{index}].deviceType")
    address = entry.get("address", "")
    if address is None:
        address = ""
    if not isinstance(address, str):
        raise ValueError(f"controllers[{index}].address must be a string")
    number = entry.get("number")
    _require_integer(number, f"controllers[{index}].number")
    supply_voltage = entry.get("supplyVoltage", entry.get("supply_voltage"))
    if supply_voltage is not None and (
        not isinstance(supply_voltage, (int, float))
        or isinstance(supply_voltage, bool)
        or supply_voltage <= 0
    ):
        raise ValueError(
            f"controllers[{index}].supplyVoltage must be a positive number or null"
        )
    return {
        "kind": kind,
        "device_type": device_type,
        "address": address.strip() or None,
        "number": number,
        "supply_voltage": (
            float(supply_voltage) if supply_voltage is not None else None
        ),
        "from_v1": False,
    }


def _validate_motor_entry(entry: Any, index: int, *, version: int) -> Dict[str, Any]:
    if not isinstance(entry, dict):
        raise ValueError(f"motors[{index}] must be an object")
    name = entry.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"motors[{index}].name must be a non-empty string")

    settings: Dict[str, Any] = {}
    aliases = {
        "pulseWidthMin": "pulse_width_min",
        "pulseWidthMax": "pulse_width_max",
        "rotationRangeMin": "rotation_range_min",
        "rotationRangeMax": "rotation_range_max",
        "turnedOn": "turned_on",
        "currentLimit": "current_limit",
        "torqueLimit": "torque_limit",
    }
    for key in MOTOR_SETTING_KEYS:
        camel = next((c for c, snake in aliases.items() if snake == key), key)
        if camel in entry:
            settings[key] = entry[camel]
        elif key in entry:
            settings[key] = entry[key]
    _validate_motor_settings(settings, index)

    if version == 1:
        pins_raw = entry.get("brickletPins", entry.get("bricklet_pins", []))
        if not isinstance(pins_raw, list):
            raise ValueError(f"motors[{index}].brickletPins must be an array")
        mappings = [
            _validate_v1_pin(pin, index, pin_index)
            for pin_index, pin in enumerate(pins_raw)
        ]
        mapping = mappings[0] if mappings else None
    else:
        number = entry.get("controllerNumber", entry.get("controller_number"))
        channel = entry.get("channel")
        if number is None and channel is None:
            mapping = None
        else:
            _require_integer(number, f"motors[{index}].controllerNumber")
            _require_integer(channel, f"motors[{index}].channel")
            mapping = {"controller_number": number, "channel": channel}

    return {"name": name.strip(), "settings": settings, "mapping": mapping}


def _validate_motor_settings(settings: Dict[str, Any], index: int) -> None:
    for key, value in settings.items():
        if key in ("turned_on", "visible", "invert"):
            if not isinstance(value, bool):
                raise ValueError(f"motors[{index}].{key} must be a boolean")
        elif key in ("current_limit", "torque_limit"):
            if value is not None and (
                not isinstance(value, (int, float)) or isinstance(value, bool)
            ):
                raise ValueError(f"motors[{index}].{key} must be a number or null")
            if value is not None:
                settings[key] = float(value)
        elif not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"motors[{index}].{key} must be an integer")


def _validate_v1_pin(entry: Any, motor_index: int, pin_index: int) -> Dict[str, int]:
    if not isinstance(entry, dict):
        raise ValueError(
            f"motors[{motor_index}].brickletPins[{pin_index}] must be an object"
        )
    number = entry.get("brickletNumber", entry.get("bricklet_number"))
    channel = entry.get("pin")
    _require_integer(
        number,
        f"motors[{motor_index}].brickletPins[{pin_index}].brickletNumber",
    )
    _require_integer(channel, f"motors[{motor_index}].brickletPins[{pin_index}].pin")
    invert = entry.get("invert", False)
    if not isinstance(invert, bool):
        raise ValueError(
            f"motors[{motor_index}].brickletPins[{pin_index}].invert must be a boolean"
        )
    return {"controller_number": number, "channel": channel}


def _require_integer(value: Any, field: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{field} must be an integer")


def _validate_device_type(value: Any, kind: str, field: str) -> None:
    if value is not None and value not in TINKERFORGE_DEVICE_TYPES:
        raise ValueError(f"{field} {value!r} is not a supported Bricklet type")
    if kind != TINKERFORGE_BRICKLET and value is not None:
        raise ValueError(f"{field} must be null for controller kind {kind!r}")


def _validate_document_references(
    controllers: List[Dict[str, Any]], motors: List[Dict[str, Any]]
) -> None:
    seen_numbers: Set[int] = set()
    seen_addresses: Set[str] = set()
    for controller in controllers:
        number = controller["number"]
        if number in seen_numbers:
            raise ValueError(f"Duplicate controller number in import: {number}")
        seen_numbers.add(number)
        address = controller["address"]
        if address:
            if address in seen_addresses:
                raise ValueError(
                    f"Duplicate controller address assignment: '{address}'"
                )
            seen_addresses.add(address)

    seen_names: Set[str] = set()
    for motor in motors:
        if motor["name"] in seen_names:
            raise ValueError(f"Duplicate motor name in import: '{motor['name']}'")
        seen_names.add(motor["name"])
        mapping = motor["mapping"]
        if mapping is None:
            continue
        number = mapping["controller_number"]
        if number in seen_numbers:
            continue
        if Controller.query.filter(Controller.number == number).first() is None:
            raise ValueError(
                f"Motor '{motor['name']}' references unknown controller number {number}"
            )


def _apply_hardware_config(document: Dict[str, Any]) -> None:
    target_numbers = [c["number"] for c in document["controllers"]]
    if target_numbers:
        Controller.query.filter(Controller.number.in_(target_numbers)).update(
            {Controller.address: None}, synchronize_session=False
        )
        db.session.flush()

    for entry in document["controllers"]:
        controller = Controller.query.filter(
            Controller.number == entry["number"]
        ).one_or_none()
        if controller is None:
            controller = Controller(
                number=entry["number"],
                kind=entry["kind"],
                device_type=entry["device_type"],
                address=entry["address"],
                supply_voltage=entry["supply_voltage"],
            )
            db.session.add(controller)
        else:
            if not entry["from_v1"]:
                if entry["kind"] != TINKERFORGE_BRICKLET:
                    controller.device_type = None
                controller.kind = entry["kind"]
                controller.supply_voltage = entry["supply_voltage"]
            controller.device_type = entry["device_type"]
            controller.address = entry["address"]
    db.session.flush()

    for entry in document["motors"]:
        motor = Motor.query.filter(Motor.name == entry["name"]).one_or_none()
        if motor is None:
            raise ValueError(f"Unknown motor '{entry['name']}'")
        for key, value in entry["settings"].items():
            setattr(motor, key, value)
        if entry["mapping"] is not None:
            motor.controller = Controller.query.filter(
                Controller.number == entry["mapping"]["controller_number"]
            ).one()
            motor.channel = entry["mapping"]["channel"]
