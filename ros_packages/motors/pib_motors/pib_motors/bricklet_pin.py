import logging
from typing import Any, Dict, Tuple
import serial
from motors.STservo_sdk import *
# -------------------------
# STS specifics & utilities
# -------------------------

# STS tick-to-degree factor from your examples (~ 4095 ticks ? 360)
_TICKS_PER_DEG = 11.378

# Reasonable defaults if not supplied via settings
_DEFAULT_BAUD = 1_000_000
_DEFAULT_SPEED = 1000
_DEFAULT_ACCEL = 50

# Cache one serial port + packet handler per device so multiple motors on the same bus reuse it
_port_cache: Dict[str, Tuple[PortHandler, any]] = {}


def _get_or_open_port(device: str, baud: int) -> Tuple[PortHandler, any]:
    """
    Returns (port_handler, packet_handler) for a serial device, opening it if needed.
    """
    if device in _port_cache:
        return _port_cache[device]

    ph = PortHandler(device)
    if not ph.openPort():
        raise RuntimeError(f"Failed to open STS port: {device}")
    if not ph.setBaudRate(baud):
        raise RuntimeError(f"Failed to set STS baudrate {baud} on {device}")

    pk = sts(ph)
    _port_cache[device] = (ph, pk)
    return ph, pk


def _deg_to_ticks(deg: float) -> int:
    # Clamp to valid STS range [0..4095] after conversion
    #ticks = int(round(((deg + 9000) * 4096 / 18000)))
    #return max(0, min(4095, ticks))
    if deg < 0:
        # Map from -9000..0 to 1000..2000
        ticks = int(round(1000 + (deg + 9000) * (1000 / 9000)))
    else:
        # Map from 0..9000 to 2000..3050
        ticks= int(round(2000 + deg * (1050 / 9000)))
    return max(1000, min(3000, ticks))

def _ticks_to_deg(ticks: int) -> float:
    return float(ticks * 18000 / 4096) - 9000

class BrickletPin:

    NO_CURRENT: int = -1

    def __init__(self, pin: int, uid: str, invert: bool) -> None:
        self.pin: int = pin
        self.uid: str = uid
        self.invert: bool = invert
        self._connected: bool | None = None
        self._baudrate: int = _DEFAULT_BAUD
        # Persist settings we can honor on STS
        self._settings: Dict[str, Any] = {
            "velocity": _DEFAULT_SPEED,
            "acceleration": _DEFAULT_ACCEL,
            "deceleration": _DEFAULT_ACCEL,  # STS has acc; decel kept for compatibility
            # The following are unsupported in STS; kept to avoid breaking callers
            "pulseWidthMin": None,
            "pulseWidthMax": None,
            "period": None,
            "turnedOn": True,
        }
        # Lazily opened/cached
        self._ph: PortHandler | None = None
        self._pk: any | None = None
        self.check_connection()

    def __str__(self) -> str:
        return f"STS-PIN[ id: {self.pin}, device: {self.uid} ]"
    
    def check_connection(self):
        """checks if the bricklet-pin is connected to a bricklet"""
        try:
            self._ph, self._pk = _get_or_open_port(self.uid, self._baudrate)
            # Probe by reading pos/speed for this ID
            _, _, res, _err = self._pk.ReadPosSpeed(self.pin)
            self._connected = (res == COMM_SUCCESS)
        except Exception:
            self._connected = False
        return bool(self._connected)

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        """apply the provided settings to the bricklet-pin"""
        if not self.is_connected():
            return False
        try:
            for key in ("velocity", "acceleration", "deceleration", "turnedOn",
                        "pulseWidthMin", "pulseWidthMax", "period"):
                if key in settings_dto:
                    self._settings[key] = settings_dto[key]
            return True
        except Exception as error:
            logging.error(
                f"error occured while trying to apply motor-settings: {str(error)}"
            )
        return False

    def get_settings(self) -> dict[str, Any]:
        return dict(self._settings)

    def get_current(self) -> int:
        """returns the current of the bricklet pin, or NO_CURRENT, if it is not connected"""
        try:
            sts_present_current, resC, errC = self._pk.ReadCurrent(self.pin)
            #if res != COMM_SUCCESS:
            #   return 0
            return sts_present_current*10
        except Exception:
            return BrickletPin.NO_CURRENT

    def set_position(self, position: int) -> bool:
        """sets the position of the bricklet-pin and returns 'True' if this was successful"""
        if not self.is_connected():
            return False
        deg = float(position)
        if self.invert:
            deg *= -1.0
        ticks = _deg_to_ticks(deg)
        speed = int(self._settings.get("velocity", _DEFAULT_SPEED))
        acc = int(self._settings.get("acceleration", _DEFAULT_ACCEL))
        
        try:
            STS3095_pins = [41, 40, 50, 51, 18, 38, 20, 21, 39, 19]
            if self.pin in STS3095_pins:
                res, err = self._pk.WritePosEx(self.pin, ticks, 700, 30)
            else:
                res, err = self._pk.WritePosEx(self.pin, ticks, 3000, 100)

            if res != COMM_SUCCESS:
                logging.error(f"STS WritePosEx failed (res={res}, err={err})")
                return False
        except Exception as e:
            logging.error(f"Exception during WritePosEx: {e}")
            return False

        return True

    def get_position(self) -> int:
        """returns the current position of the bricklet-pin, or '0' if not connected to a bricklet"""
        if not self.is_connected():
            return 0
        try:
            ticks, _spd, res, _err = self._pk.ReadPosSpeed(self.pin)
            if res != COMM_SUCCESS:
                return 0
            deg = _ticks_to_deg(int(ticks))
            return int(round(deg))
        except Exception:
            return 0
