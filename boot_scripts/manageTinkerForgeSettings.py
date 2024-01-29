#This python script contains methods to get and save the UIDs from tinkerforge in an external file
#Also it contains method to update the UDIs in the database and to compare the IDs with the database
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT
import requests
import json

class TinkerForge:
        UID0 = "X"
        UID1 = "Y"
        UID2 = "Z"

        def getIDs(self):
            self.ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, self.cb_enumerate)
            self.ipcon.enumerate()
            self.printAllUIDs()
            return [self.UID0, self.UID1, self.UID2]

        def cb_enumerate(self, uid, connected_uid, position, hardware_version, firmware_version, device_identifier, enumeration_type):
            if position == "a":
                    self.UID0 = uid
            if position == "b":
                    self.UID1 = uid
            if position == "e":
                    self.UID2 = uid

        def getValue(cls, number):
            variable_name = "UID" + str(number)
            return getattr(cls, variable_name, None)
        
        def printAllUIDs(self):
                print("UID0 = " + self.UID0)
                print("UID1 = " + self.UID1)
                print("UID2 = " + self.UID2)

        def __init__(self):
            self.ipcon = IPConnection()
            self.hat = BrickHAT("X", self.ipcon)
            self.ipcon.connect("localhost", 4223)
            pass
        
tinker = TinkerForge()

def updateUIDs():
    tinker.getIDs()
    header = {"Content-Type": "application/json"}
    response0 = requests.put("http://127.0.0.1:5000//bricklet/1", data=json.dumps({"uid": tinker.UID0}), headers=header)
    response1 = requests.put("http://127.0.0.1:5000//bricklet/2", data=json.dumps({"uid": tinker.UID1}), headers=header)
    response2 = requests.put("http://127.0.0.1:5000//bricklet/3", data=json.dumps({"uid": tinker.UID2}), headers=header)
    return

def getUIDsFromDB():
      response = requests.get("http://127.0.0.1:5000//bricklet")
      json_data = json.loads(response.text)
      return [json_data['bricklets'][0]['uid'], json_data['bricklets'][1]['uid'], json_data['bricklets'][2]['uid']]
    

def changesBetweenCurrentAndTinkerForgeUIDs():
      usedUIDs = getUIDsFromDB()
      tinker.getIDs()
      for i in range(len(usedUIDs)):
            if(usedUIDs[i] != tinker.getValue(i)):
                return True
      return False
