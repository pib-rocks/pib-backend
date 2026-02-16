"""
STS-compatible replacement for the old Tinkerforge bricklet layer.
Keeps same interface but uses STS BrickletPin instead of Tinkerforge bricklets.
"""

import logging
from types import SimpleNamespace

from pib_api_client import bricklet_client
from pib_motors.config import cfg

from pib_motors.bricklet_pin import BrickletPin  # <-- your STS-based class

TINKERFORGE_HOST = "localhost"
TINKERFORGE_PORT = 4223

# No real Tinkerforge connection
BrickHAT = BrickletServoV2 = BrickletSolidStateRelayV2 = IPConnection = object
Error = SimpleNamespace()

# --- Retrieve bricklets from pib-api (still works) ---
successful, bricklet_dtos = bricklet_client.get_all_bricklets()
if not successful or "bricklets" not in bricklet_dtos:
    logging.warning("?? Failed to load bricklets from pib-api; using empty set.")
    bricklet_dtos = {"bricklets": []}

servo_bricklet_uids = []
solid_state_relay_bricklet_uid = None

for dto in bricklet_dtos["bricklets"]:
    if dto["type"] == "Servo Bricklet":
        servo_bricklet_uids.append(dto["uid"])
    elif dto["type"] == "Solid State Relay Bricklet" and dto["uid"]:
        solid_state_relay_bricklet_uid = dto["uid"]

# --- Initialize STS BrickletPin objects instead of Tinkerforge ones ---
uid_to_servo_bricklet: dict[str, BrickletPin] = {}
for uid in servo_bricklet_uids:
    try:
        # uid is your STS device path (e.g. "/dev/ttyUSB0")
        uid_to_servo_bricklet[uid] = BrickletPin(pin=1, uid=uid, invert=False)
        logging.info(f"Initialized STS motor on {uid}")
    except Exception as e:
        logging.error(f"Failed to initialize STS motor for {uid}: {e}")

# --- Handle SSR gracefully (optional, log only) ---
solid_state_relay_bricklet = None
def set_ssr_state(state: bool) -> None:
    logging.info(f"[mock] Solid-State Relay {'ON' if state else 'OFF'} (no-op)")

# --- Connected bricklets tracker ---
connected_bricklets = set(uid_to_servo_bricklet.keys())

def connected_enumerate(*_, **__):
    logging.debug("[mock] STS enumeration event ignored")
