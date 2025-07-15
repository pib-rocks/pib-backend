import os
import time
import logging
from pib_api_client import bricklet_client
from pib_motors.config import cfg
from tinkerforge.brick_hat import BrickHAT
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.bricklet_air_quality import BrickletAirQuality
from tinkerforge.ip_connection import IPConnection

TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", 4223))
BRICKLET_AQ_UID = "2eed"

# Connection
ipcon = IPConnection()  # Create IP connection
hat = BrickHAT("X", ipcon)
ipcon.connect(cfg.TINKERFORGE_HOST, cfg.TINKERFORGE_PORT)

# get data from pib-api
for attempt in range(1, 5):
    successful, bricklet_dtos = bricklet_client.get_all_bricklets()
    if successful:
        break
    logging.warning(f"failed to load bricklets from pib-api, retry {attempt}..")
    time.sleep(2)
else:
    raise RuntimeError("failed to load bricklets from pib-api...")
bricklet_uids = [dto["uid"] for dto in bricklet_dtos["bricklets"]]

# maps the uid (e.g. 'XYZ') to the associated bricklet object
uid_to_bricklet: dict[str, BrickletServoV2] = {
    uid: BrickletServoV2(uid, ipcon) for uid in bricklet_uids
}

bricklet_aq: BrickletAirQuality = BrickletAirQuality(BRICKLET_AQ_UID, ipcon)


def get_aq_bricklet_data():
    iaq_index, iaq_index_accuracy, temperature, humidity, air_pressure = (
        bricklet_aq.get_all_values()
    )

    temperature_c = temperature / 100.0
    humidity_pc = humidity / 100.0
    air_pressure_hpa = air_pressure / 100.0

    xml_data = (
        f"<WData>"
        f"<Raumtemperatur>{temperature_c:.2f}° C</Raumtemperatur>"
        f"<RelativeLuftfeuchtigkeit>{humidity_pc:.2f}% RH</RelativeLuftfeuchtigkeit>"
        f"<Luftdruck>{air_pressure_hpa:.2f} hPA</Luftdruck>"
        f"</WData>"
    )

    logging.info(f"Air Quality Data: {xml_data}")
    return xml_data
