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

uid0 = "AAA"
uid1 = "BBB"
uid2 = "CCC"
POSITION_TO_UID_MAP = {"a": "uid0", "b": "uid1", "e": "uid2"}

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
    print("update")
    """Update bricklet UIDs in the database."""
    header = {"Content-Type": "application/json"}

    for uid_number, uid in enumerate([uid0, uid1, uid2]):
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


def no_uids_in_database():
    """Check for changes between current databse and TinkerForge UIDs."""
    return get_uids_from_db() == ["AAA", "BBB", "CCC"]


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("A method parameter is required")
    else:
        methode = sys.argv[1]
        if methode == "update_uids":
            update_uids()
        elif methode == "get_uids_from_db":
            get_uids_from_db()
        elif methode == "no_uids_in_database":
            no_uids_in_database()
        elif methode == "check_and_update":
            if no_uids_in_database():
                update_uids()
        else:
            print("Method not found")
