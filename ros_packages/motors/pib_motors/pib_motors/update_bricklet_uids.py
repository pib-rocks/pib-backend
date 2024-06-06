"""Script for managing TinkerForge UIDs and corresponding database operations"""

import json
import os
import sys
import multiprocessing
import requests
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT

BASE_URL = os.getenv("FLASK_API_BASE_URL", "http://127.0.0.1:5000")
TINKERFORGE_HOST = os.getenv("TINKERFORGE_HOST", "localhost")
TINKERFORGE_PORT = int(os.getenv("TINKERFORGE_PORT", 4223))
BRICKLET_URLS = [f"{BASE_URL}/bricklet/{i}" for i in range(1, 4)]

UID0 = "X"
UID1 = "Y"
UID2 = "Z"
POSITION_TO_UID_MAP = {"a": "UID0", "b": "UID1", "e": "UID2"}

ipcon: IPConnection = IPConnection()
hat = BrickHAT("X", ipcon)
ipcon.connect(TINKERFORGE_HOST, TINKERFORGE_PORT)


def cb_enumerate(
    uid,
    connected_uid,
    position,
    hardware_version,
    firmware_version,
    device_identifier,
    enumeration_type,
):
    """Readout the UIDs of the connected TinkerForge Bricklets and update global variables."""
    if position in POSITION_TO_UID_MAP:
        globals()[POSITION_TO_UID_MAP[position]] = uid


ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, cb_enumerate)

p = multiprocessing.Process(target=lambda: ipcon.enumerate())
p.start()
p.join()
ipcon.disconnect()


def update_uids():
    """Update bricklet UIDs in the database."""
    header = {"Content-Type": "application/json"}

    for uid_number, uid in enumerate([UID0, UID1, UID2]):
        url = BRICKLET_URLS[uid_number]
        requests.put(url, data=json.dumps({"uid": uid}), headers=header)


def get_uids_from_db():
    """Retrieve all UIDs from the database."""
    response = requests.get(BASE_URL + "/bricklet")
    json_data = json.loads(response.text)
    return [
        json_data["bricklets"][0]["uid"],
        json_data["bricklets"][1]["uid"],
        json_data["bricklets"][2]["uid"],
    ]


def detect_uid_changes():
    """Check for changes between current databse and TinkerForge UIDs."""
    used_uids = get_uids_from_db()

    for uid_number, uid in enumerate([UID0, UID1, UID2]):
        if uid != used_uids[uid_number]:
            return True
    return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("A method parameter is required")
    else:
        methode = sys.argv[1]
        if methode == "update_uids":
            update_uids()
        elif methode == "get_uids_from_db":
            get_uids_from_db()
        elif methode == "detect_uid_changes":
            detect_uid_changes()
        else:
            print("Method not found")
