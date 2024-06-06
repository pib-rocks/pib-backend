import logging
from typing import Any

from pib_motors.bricklet import uid_to_bricklet
from tinkerforge.bricklet_servo_v2 import BrickletServoV2


class BrickletPin:

    NO_CURRENT: int = -1
    DEFAULT_BRICKLET_UIDS = ["AAA", "BBB", "CCC"]

    def __init__(self, pin: int, uid: str, invert: bool) -> None:
        self.pin: int = pin
        self.bricklet: BrickletServoV2 = uid_to_bricklet[uid]
        self.invert = invert

    def __str__(self) -> str:
        uid = "---"
        try:
            uid = self.bricklet.get_identity()[0]
        except Exception:
            pass
        return f"BRICKLET-PIN[ pin: {self.pin}, bricklet: {uid} ]"

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
        return (
            self.bricklet.get_servo_current(self.pin)
            if self.is_connected()
            else BrickletPin.NO_CURRENT
        )

    def is_connected(self) -> bool:
        """checks if the bricklet-pin is connected to a bricklet"""
        return self.bricklet.uid_string not in BrickletPin.DEFAULT_BRICKLET_UIDS

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
