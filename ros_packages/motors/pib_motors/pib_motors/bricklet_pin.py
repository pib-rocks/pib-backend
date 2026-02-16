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
    """
    STS-backed implementation compatible with the original BrickletPin interface.

    Mapping:
      - pin:  STS servo ID (int)
      - uid:  serial device path, e.g., '/dev/ttyMotor0'
      - invert: same semantics as before

    Methods kept: check_connection, apply_settings, get_settings, get_current,
                  is_connected, set_position, get_position
    """

    NO_CURRENT: int = -1

    def __init__(self, pin: int, uid: str, invert: bool) -> None:
        """
        pin  -> STS servo ID
        uid  -> serial device (e.g., '/dev/ttyMotor0')
        """
        self.pin: int = int(pin)             # STS ID
        self.uid: str = uid                  # serial device
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

        # Attempt initial check/open
        self.check_connection()

    def __str__(self) -> str:
        return f"STS-PIN[ id: {self.pin}, device: {self.uid} ]"

    # ----------------
    # Connection state
    # ----------------
    def check_connection(self) -> bool:
        """Check we can talk to the STS bus and read this ID once."""
        try:
            self._ph, self._pk = _get_or_open_port(self.uid, self._baudrate)
            # Probe by reading pos/speed for this ID
            _, _, res, _err = self._pk.ReadPosSpeed(self.pin)
            self._connected = (res == COMM_SUCCESS)
        except Exception:
            self._connected = False
        return bool(self._connected)

    def is_connected(self) -> bool:
        if self._connected is None:
            return self.check_connection()
        return bool(self._connected)

    # -------------
    # Settings I/O
    # -------------
    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        """
        Apply compatible settings to STS:
          - velocity  -> used with WritePosEx
          - acceleration -> used with WritePosEx
          - deceleration -> stored (STS doesn't separate, we reuse 'acceleration')
          - turnedOn -> stored (STS has torque enable in some models; not used here)
        Other Tinkerforge-specific fields are accepted but ignored safely.
        """
        if not self.is_connected():
            return False

        try:
            # Merge known settings; ignore unknown keys gracefully
            for key in ("velocity", "acceleration", "deceleration", "turnedOn",
                        "pulseWidthMin", "pulseWidthMax", "period"):
                if key in settings_dto:
                    self._settings[key] = settings_dto[key]

            # Nothing to actively send until a position command (STS sets speed/acc there)
            return True
        except Exception as error:
            logging.error(f"Error while applying STS motor settings: {error}")
            return False

    def get_settings(self) -> dict[str, Any]:
        """
        Return the last applied/known settings (best-effort parity with old API).
        """
        return dict(self._settings)

    # --------
    # Telemetry
    # --------
    def get_current(self) -> int:
        try:
            sts_present_current, resC, errC = self._pk.ReadCurrent(self.pin)
            #if res != COMM_SUCCESS:
            #   return 0
            return sts_present_current*10
        except Exception:
            return BrickletPin.NO_CURRENT
    # -------------
    # Position I/O
    # -------------
    def set_position(self, position: int) -> bool:
        """
        Set target position in DEGREES (kept consistent with your examples).
        If your upstream publishes a different unit, adjust mapping here.
        """
        if not self.is_connected():
            return False

        deg = float(position)
        if self.invert:
            deg *= -1.0

        ticks = _deg_to_ticks(deg)
        speed = int(self._settings.get("velocity", _DEFAULT_SPEED))
        acc = int(self._settings.get("acceleration", _DEFAULT_ACCEL))
        '''
        try:
            if self.pin == 41 or self.pin == 40 or self.pin == 50 or self.pin == 51 or self.pin == 41 or self.pin == 18 or self.pin == 38 or self.pin == 20 or self.pin == 21:
                res, err = self._pk.WritePosEx(self.pin, ticks, 1200, 30) #_default/20
            
            if res != COMM_SUCCESS:
                logging.error(f"STS WritePosEx failed (res={res}, err={err})")
                return False
            
            else:
                res, err = self._pk.WritePosEx(self.pin, ticks, 3000, 30) #_default/20
                
        except Exception as e:
            logging.error(f"Exception during WritePosEx: {e}")
            return False
        return True
        '''
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
        """
        Read current position in DEGREES (rounded to int for API parity).
        """
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
