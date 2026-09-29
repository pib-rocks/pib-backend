import os
import logging
import time
from typing import Any

from pib_api_client import bricklet_client
from pib_motors.bricklet_mapping import select_bricklet_uids
from pib_motors.config import cfg
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton
from tinkerforge.ip_connection import IPConnection, Error

TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", 4223))

# Connection
ipcon = IPConnection()  # Create IP connection
hat = BrickHAT("X", ipcon)
ipcon.connect(cfg.TINKERFORGE_HOST, cfg.TINKERFORGE_PORT)

# get data from pib-api
BRICKLET_LOAD_RETRY_INTERVAL_SECONDS = 2.0
BRICKLET_LOAD_TIMEOUT_SECONDS = 60.0


def load_bricklets(
    retry_interval_seconds: float = BRICKLET_LOAD_RETRY_INTERVAL_SECONDS,
    timeout_seconds: float = BRICKLET_LOAD_TIMEOUT_SECONDS,
) -> dict[str, Any]:
    """Load the bricklet configuration from pib-api, retrying while it starts up.

    After a reboot this module is imported before pib-api accepts connections,
    so a single refused request must not be fatal for the motor_control node.
    """
    attempts = max(1, int(timeout_seconds // retry_interval_seconds))

    for attempt in range(1, attempts + 1):
        try:
            successful, bricklet_dtos = bricklet_client.get_all_bricklets()
        except Exception as error:
            successful, bricklet_dtos = False, None
            reason = f"{type(error).__name__}: {error}"
        else:
            reason = "pib-api did not return a bricklet configuration"

        if successful:
            if attempt > 1:
                logging.info(f"loaded bricklets from pib-api on attempt {attempt}")
            return bricklet_dtos

        if attempt < attempts:
            logging.warning(
                f"could not load bricklets from pib-api ({reason}) - "
                f"attempt {attempt}/{attempts}, "
                f"retrying in {retry_interval_seconds}s..."
            )
            time.sleep(retry_interval_seconds)

    raise RuntimeError("failed to load bricklets from pib-api...")


bricklet_dtos = load_bricklets()

servo_bricklet_uids = []
solid_state_relay_bricklet_uid = None
rgb_led_bricklet_uids = []

for dto in bricklet_dtos["bricklets"]:
    if dto["type"] == "Servo Bricklet":
        servo_bricklet_uids.append(dto["uid"])
    elif dto["type"] == "Solid State Relay Bricklet" and dto["uid"]:
        solid_state_relay_bricklet_uid = dto["uid"]
    elif dto["type"] == "RGB LED Button Bricklet" and dto["uid"]:
        rgb_led_bricklet_uids.append(dto["uid"])


def build_bricklets(
    bricklet_class, uids: list[Any], device_type: str
) -> dict[str, Any]:
    """Map every usable uid to its bricklet object, skipping the rest.

    One misconfigured uid must cost exactly that bricklet - it must never take
    the whole node down, because the motor, relay and button nodes all import
    this module.
    """
    selection = select_bricklet_uids(uids)
    for skipped in selection.skipped:
        logging.error(
            f"skipping {device_type} '{skipped.uid}': {skipped.reason} - "
            "this device stays unavailable until its UID is corrected"
        )

    bricklets: dict[str, Any] = {}
    for uid in selection.valid:
        try:
            bricklets[uid] = bricklet_class(uid, ipcon)
        except Error as error:
            logging.error(
                f"skipping {device_type} '{uid}': rejected by tinkerforge "
                f"({error}) - this device stays unavailable"
            )
    return bricklets


# maps the uid (e.g. 'XYZ') to the associated servo bricklet object
uid_to_servo_bricklet: dict[str, BrickletServoV2] = build_bricklets(
    BrickletServoV2, servo_bricklet_uids, "Servo Bricklet"
)

# maps the uid (e.g. 'XYZ') to the associated solid state relay bricklet object
_solid_state_relay_bricklets = build_bricklets(
    BrickletSolidStateRelayV2,
    [solid_state_relay_bricklet_uid],
    "Solid State Relay Bricklet",
)
solid_state_relay_bricklet: BrickletSolidStateRelayV2 | None = next(
    iter(_solid_state_relay_bricklets.values()), None
)
if solid_state_relay_bricklet is not None:
    solid_state_relay_bricklet.set_response_expected(
        BrickletSolidStateRelayV2.FUNCTION_SET_STATE, True
    )

# maps the uid (e.g. 'XYZ') to the associated rgb led button bricklet object
uid_to_rgb_led_bricklet: dict[str, BrickletRGBLEDButton] = build_bricklets(
    BrickletRGBLEDButton, rgb_led_bricklet_uids, "RGB LED Button Bricklet"
)


def set_ssr_state(state: bool) -> None:
    """set the status of the solid state relay to on (true) or off (false)"""
    if solid_state_relay_bricklet is None:
        raise RuntimeError("UID of Solid-State Relay is not set.")
    else:
        try:
            solid_state_relay_bricklet.set_state(state)
            logging.info(f"Solid-State Relay {'ON' if state else 'OFF'}")
        except Error as e:
            if e.value == Error.TIMEOUT or e.value == Error.WRONG_RESPONSE_LENGTH:
                logging.error(
                    "Solid-State Relay is not connected or unresponsive (Timeout). Please check the UID and ensure the device is plugged in."
                )
            elif e.value == Error.INVALID_UID:
                logging.error("Invalid UID for Solid-State Relay.")
            elif e.value == Error.NOT_CONNECTED:
                logging.error("No connection to Solid-State Relay.")
            else:
                logging.error(f"Unknown Tinkerforge error: {e}")
            raise e


connected_bricklets = set()


# callback function to handle bricklet enumeration events
def connected_enumerate(
    uid,
    connected_uid,
    position,
    hardware_version,
    firmware_version,
    device_identifier,
    enumeration_type,
):
    if (
        device_identifier == BrickletServoV2.DEVICE_IDENTIFIER
        and enumeration_type == IPConnection.ENUMERATION_TYPE_AVAILABLE
    ):
        connected_bricklets.add(uid)
        logging.info(f"Servo Bricklet {uid} is connected.")
    elif enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
        connected_bricklets.discard(uid)
        logging.info(f"Servo Bricklet {uid} is disconnected.")
