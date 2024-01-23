from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from pib_api_client import bricklet_client

HOST = "localhost"
PORT = 4223

# Connection
ipcon = IPConnection()  # Create IP connection
hat = BrickHAT("X", ipcon)
ipcon.connect(HOST, 4223)

# get data from pib-api
successful, bricklet_dtos = bricklet_client.get_all_bricklets()
if not successful: raise Exception("failed to load bricklets from pib-api...")
bricklet_uids = [ dto['uid'] for dto in bricklet_dtos['bricklets'] ]

# maps the uid (e.g. 'XYZ') to the associated bricklet object
uid_to_bricklet: dict[str, BrickletServoV2] = { uid : BrickletServoV2(uid, ipcon) for uid in bricklet_uids }