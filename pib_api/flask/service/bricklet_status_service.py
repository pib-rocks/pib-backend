import logging
import os
from typing import Any, Dict, List, Optional

from sqlalchemy.orm import joinedload

from model.bricklet_model import Bricklet
from model.bricklet_pin_model import BrickletPin
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2
from tinkerforge.ip_connection import Error, IPConnection


logger = logging.getLogger(__name__)

TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", "4223"))


def _identity_dict(device) -> Optional[Dict[str, Any]]:
    try:
        uid, connected_uid, position, hw, fw, device_id = device.get_identity()
        return {
            "uid": uid,
            "connectedUid": connected_uid,
            "position": position,
            "hardwareVersion": list(hw),
            "firmwareVersion": list(fw),
            "deviceIdentifier": device_id,
        }
    except Error as exc:
        logger.warning("Unable to read identity: %s", exc)
        return None


def _servo_pin_status(
    servo: BrickletServoV2, pin: int, motor_name: Optional[str], invert: bool
) -> Dict[str, Any]:
    status: Dict[str, Any] = {
        "pin": pin,
        "motorName": motor_name,
        "invert": invert,
        "connected": False,
        "enabled": None,
        "currentMa": None,
        "position": None,
        "voltageMv": None,
    }
    try:
        current = servo.get_servo_current(pin)
        status["connected"] = True
        status["currentMa"] = current
        status["enabled"] = servo.get_enabled(pin)
        status["position"] = servo.get_position(pin)
    except Error as exc:
        logger.debug("Servo pin %s unavailable: %s", pin, exc)
        return status

    try:
        status["voltageMv"] = servo.get_input_voltage()
    except Error:
        status["voltageMv"] = None
    return status


def _relay_status(relay: BrickletSolidStateRelayV2) -> Dict[str, Any]:
    try:
        return {"turnedOn": relay.get_state(), "connected": True}
    except Error as exc:
        logger.debug("Relay unavailable: %s", exc)
        return {"turnedOn": None, "connected": False}


def _rgb_button_status(button: BrickletRGBLEDButton) -> Dict[str, Any]:
    status: Dict[str, Any] = {
        "connected": False,
        "color": None,
        "buttonState": None,
        "identity": None,
    }
    try:
        r, g, b = button.get_color()
        button_state = button.get_button_state()
        status["connected"] = True
        status["color"] = {"r": r, "g": g, "b": b}
        status["buttonState"] = (
            "pressed"
            if button_state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED
            else "released"
        )
        status["identity"] = _identity_dict(button)
    except Error as exc:
        logger.debug("RGB button unavailable: %s", exc)
    return status


def _with_ip_connection(callback):
    ipcon = IPConnection()
    try:
        ipcon.connect(TINKERFORGE_HOST, TINKERFORGE_PORT)
        return callback(ipcon)
    except Exception as exc:
        logger.warning("Tinkerforge connection failed: %s", exc)
        return None
    finally:
        try:
            ipcon.disconnect()
        except Exception:
            pass


def get_bricklets_status() -> Dict[str, Any]:
    bricklets: List[Bricklet] = (
        Bricklet.query.options(
            joinedload(Bricklet.bricklet_pins).joinedload(BrickletPin.motor)
        )
        .order_by(Bricklet.bricklet_number)
        .all()
    )

    def collect(ipcon: IPConnection) -> List[Dict[str, Any]]:
        result: List[Dict[str, Any]] = []
        for bricklet in bricklets:
            entry: Dict[str, Any] = {
                "brickletNumber": bricklet.bricklet_number,
                "uid": bricklet.uid,
                "type": bricklet.type,
                "connected": False,
                "identity": None,
                "servo": None,
                "relay": None,
                "rgbButton": None,
            }

            if not bricklet.uid:
                result.append(entry)
                continue

            if bricklet.type == "Servo Bricklet":
                servo = BrickletServoV2(bricklet.uid, ipcon)
                entry["identity"] = _identity_dict(servo)
                pins = []
                for bricklet_pin in bricklet.bricklet_pins:
                    motor_name = bricklet_pin.motor.name if bricklet_pin.motor else None
                    pins.append(
                        _servo_pin_status(
                            servo,
                            bricklet_pin.pin,
                            motor_name,
                            bricklet_pin.invert,
                        )
                    )
                voltage = next(
                    (p["voltageMv"] for p in pins if p.get("voltageMv") is not None),
                    None,
                )
                any_connected = (
                    any(p["connected"] for p in pins) or entry["identity"] is not None
                )
                if not pins and entry["identity"] is not None:
                    any_connected = True
                    try:
                        voltage = servo.get_input_voltage()
                    except Error:
                        voltage = None
                entry["connected"] = any_connected
                entry["servo"] = {"pins": pins, "voltageMv": voltage}
            elif bricklet.type == "Solid State Relay Bricklet":
                relay = BrickletSolidStateRelayV2(bricklet.uid, ipcon)
                entry["identity"] = _identity_dict(relay)
                entry["relay"] = _relay_status(relay)
                entry["connected"] = bool(entry["relay"].get("connected"))
            elif bricklet.type == "RGB LED Button Bricklet":
                button = BrickletRGBLEDButton(bricklet.uid, ipcon)
                rgb = _rgb_button_status(button)
                entry["rgbButton"] = rgb
                entry["identity"] = rgb.get("identity")
                entry["connected"] = bool(rgb.get("connected"))

            result.append(entry)
        return result

    live = _with_ip_connection(collect)
    if live is not None:
        return {"bricklets": live, "tinkerforgeConnected": True}

    fallback = []
    for bricklet in bricklets:
        entry = {
            "brickletNumber": bricklet.bricklet_number,
            "uid": bricklet.uid,
            "type": bricklet.type,
            "connected": False,
            "identity": None,
            "servo": None,
            "relay": None,
            "rgbButton": None,
        }
        if bricklet.type == "Servo Bricklet":
            entry["servo"] = {
                "pins": [
                    {
                        "pin": bp.pin,
                        "motorName": bp.motor.name if bp.motor else None,
                        "invert": bp.invert,
                        "connected": False,
                        "enabled": None,
                        "currentMa": None,
                        "position": None,
                        "voltageMv": None,
                    }
                    for bp in bricklet.bricklet_pins
                ],
                "voltageMv": None,
            }
        elif bricklet.type == "Solid State Relay Bricklet":
            entry["relay"] = {"turnedOn": None, "connected": False}
        elif bricklet.type == "RGB LED Button Bricklet":
            entry["rgbButton"] = {
                "connected": False,
                "color": None,
                "buttonState": None,
                "identity": None,
            }
        fallback.append(entry)
    return {"bricklets": fallback, "tinkerforgeConnected": False}
