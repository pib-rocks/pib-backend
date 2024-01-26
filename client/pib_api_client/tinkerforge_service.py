from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_hat import BrickHAT

class TinkerForge:
        UID0 = "X"
        UID1 = "Y"
        UID2 = "Z"

        def get_ids(self):
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