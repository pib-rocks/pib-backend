import os
import logging

from pib_api_client import bricklet_client
from pib_motors.config import cfg
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2
from tinkerforge.ip_connection import IPConnection, Error

TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", 4223))

# Connection
ipcon = IPConnection()  # Create IP connection
hat = BrickHAT("X", ipcon)
ipcon.connect(cfg.TINKERFORGE_HOST, cfg.TINKERFORGE_PORT)

# get data from pib-api
successful, bricklet_dtos = bricklet_client.get_all_bricklets()
if not successful:
    raise RuntimeError("failed to load bricklets from pib-api...")

servo_bricklet_uids = []
solid_state_relay_bricklet_uid = None

for dto in bricklet_dtos["bricklets"]:
    if dto["type"] == "Servo Bricklet":
        servo_bricklet_uids.append(dto["uid"])
    elif dto["type"] == "Solid State Relay Bricklet" and dto["uid"]:
        solid_state_relay_bricklet_uid = dto["uid"]

# maps the uid (e.g. 'XYZ') to the associated servo bricklet object
uid_to_servo_bricklet: dict[str, BrickletServoV2] = {
    uid: BrickletServoV2(uid, ipcon) for uid in servo_bricklet_uids if uid
}

# maps the uid (e.g. 'XYZ') to the associated solid state relay bricklet object
if solid_state_relay_bricklet_uid:
    solid_state_relay_bricklet: BrickletSolidStateRelayV2 = BrickletSolidStateRelayV2(
        solid_state_relay_bricklet_uid, ipcon
    )


def set_ssr_state(state: bool) -> None:
    """set the status of the solid state relay to on (true) or off (false)"""
    if solid_state_relay_bricklet_uid is None:
        logging.error("The UID of the Solid-State Relay is not set.")
        return
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
