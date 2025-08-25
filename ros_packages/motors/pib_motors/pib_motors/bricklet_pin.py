import logging
from typing import Any

from pib_motors.bricklet import uid_to_servo_bricklet
from tinkerforge.bricklet_servo_v2 import BrickletServoV2
from tinkerforge.ip_connection import Error


class BrickletPin:

    NO_CURRENT: int = -1

    def __init__(self, pin: int, uid: str, invert: bool) -> None:
        self.pin: int = pin
        self.uid = uid
        self.bricklet: BrickletServoV2 | None = uid_to_servo_bricklet.get(uid)
        self.invert = invert
        self._connected: bool | None = None

        self.check_connection()

    def __str__(self) -> str:
        return f"BRICKLET-PIN[ pin: {self.pin}, bricklet: {self.uid} ]"

    def check_connection(self):
        """checks if the bricklet-pin is connected to a bricklet"""
        try:
            if self.bricklet is None:
                self._connected = False
            else:
                self.bricklet.get_servo_current(self.pin)
                self._connected = True
        except Error as e:
            self._connected = False
        return self._connected

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        """apply the provided settings to the bricklet-pin"""
        if not self.is_connected():
            return False
        try:
            self.bricklet.set_pulse_width(
                self.pin, settings_dto["pulseWidthMin"], settings_dto["pulseWidthMax"]
            )
            self.bricklet.set_motion_configuration(
                self.pin,
                settings_dto["velocity"],
                settings_dto["acceleration"],
                settings_dto["deceleration"],
            )
            self.bricklet.set_period(self.pin, settings_dto["period"])
            self.bricklet.set_enable(self.pin, settings_dto["turnedOn"])
            return True
        except Exception as error:
            logging.error(
                f"error occured while trying to apply motor-settings: {str(error)}"
            )
        return False

    def get_settings(self) -> dict[str, Any]:
        """returns the current settings of the bricklet-pin"""
        settings_dto = {}
        if not self.is_connected():
            return settings_dto
        try:
            settings_dto["pulseWidthMin"], settings_dto["pulseWidthMax"] = (
                self.bricklet.get_pulse_width(self.pin)
            )
            (
                settings_dto["velocity"],
                settings_dto["acceleration"],
                settings_dto["deceleration"],
            ) = self.bricklet.get_motion_configuration(self.pin)
            settings_dto["period"] = self.bricklet.get_period(self.pin)
            settings_dto["turnedOn"] = self.bricklet.get_enabled(self.pin)
        except Exception as error:
            logging.error(
                f"error occured while trying to get motor-settings: {str(error)}"
            )
        return settings_dto

    def get_current(self) -> int:
        """returns the current of the bricklet pin, or NO_CURRENT, if it is not connected"""
        if not self.is_connected():
            return BrickletPin.NO_CURRENT
        return self.bricklet.get_servo_current(self.pin)

    def is_connected(self) -> bool:
        """checks if the bricklet-pin is connected to a bricklet"""
        if self._connected is None:
            return self.check_connection()
        return self._connected

    def set_position(self, position: int) -> bool:
        """sets the position of the bricklet-pin and returns 'True' if this was successful"""
        if not self.is_connected():
            return False
        if self.invert:
            position *= -1
        try:
            self.bricklet.set_position(self.pin, position)
        except Exception:
            return False
        return True

    def get_position(self) -> int:
        """returns the current position of the bricklet-pin, or '0' if not connected to a bricklet"""
        if not self.is_connected():
            return 0
        try:
            return self.bricklet.get_position(self.pin)
        except Exception:
            return 0
