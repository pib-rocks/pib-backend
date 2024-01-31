from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT
import requests
import json
import sys
import multiprocessing

UID0 = "X"
UID1 = "Y"
UID2 = "Z"


ipcon = IPConnection()
hat = BrickHAT("X", ipcon)
ipcon.connect("localhost", 4223)

def cb_enumerate(uid, connected_uid, position, hardware_version, firmware_version, device_identifier, enumeration_type):
    global UID0
    global UID1
    global UID2
    if position == "a": UID0 = uid
    if position == "b": UID1 = uid
    if position == "e": UID2 = uid

ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, cb_enumerate)

def i():
    ipcon.enumerate()

p = multiprocessing.Process(target=i)
p.start()
p.join()
ipcon.disconnect()

def update_uids():
    header = {"Content-Type": "application/json"}
    response0 = requests.put("http://127.0.0.1:5000//bricklet/1", data=json.dumps({"uid": UID0}), headers=header)
    response1 = requests.put("http://127.0.0.1:5000//bricklet/2", data=json.dumps({"uid": UID1}), headers=header)
    response2 = requests.put("http://127.0.0.1:5000//bricklet/3", data=json.dumps({"uid": UID2}), headers=header)

def get_uids_from_db():
      response = requests.get("http://127.0.0.1:5000//bricklet")
      json_data = json.loads(response.text)
      return [json_data['bricklets'][0]['uid'], json_data['bricklets'][1]['uid'], json_data['bricklets'][2]['uid']]
    

def changes_between_current_and_tinker_forge_uids():
    usedUIDs = get_uids_from_db()
    if UID0 != usedUIDs[0]:
        return True
    if UID1 != usedUIDs[1]:
        return True
    if UID2 != usedUIDs[2]:
        return True
    return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Bitte geben Sie eine Methode an.")

    methode = sys.argv[1]
    if methode == "update_uids":
        update_uids()
    elif methode == "get_uids_from_db":
        get_uids_from_db()
    elif methode == "changes_between_current_and_tinker_forge_uids":
        changes_between_current_and_tinker_forge_uids()
    else:
        print("Ungueltige Methode.")
