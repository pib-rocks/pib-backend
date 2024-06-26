import os

from pib_api_client import bricklet_client
from pib_motors.config import cfg
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.ip_connection import IPConnection

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
bricklet_uids = [dto["uid"] for dto in bricklet_dtos["bricklets"]]

# maps the uid (e.g. 'XYZ') to the associated bricklet object
uid_to_bricklet: dict[str, BrickletServoV2] = {
    uid: BrickletServoV2(uid, ipcon) for uid in bricklet_uids
}
