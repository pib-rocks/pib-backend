"""Feetech STS serial bus actuator family (PC-125).

Drives Feetech STS servos attached to Waveshare USB-serial adapters on a
``pib5advanced`` robot without touching ``ServoBrickletActuator`` or the
Tinkerforge stack.
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Any

from pib_motors.stservo_sdk.port_handler import PortHandler
from pib_motors.stservo_sdk.sts import (
    STS_PRESENT_TEMPERATURE,
    STS_TORQUE_ENABLE,
    sts,
)
from pib_motors.stservo_sdk.stservo_def import COMM_SUCCESS

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_SPEED = 1000
DEFAULT_ACCEL = 50

# Heavy arm/head servos (STS3095) run at lower speed/acceleration than finger servos.
STS3095_SERVO_IDS: frozenset[int] = frozenset({18, 19, 20, 21, 38, 39, 40, 41, 50, 51})
STS3095_SPEED = 700
STS3095_ACCEL = 30
FINGER_SPEED = 3000
FINGER_ACCEL = 100

SERIAL_BY_ID_DIR = Path("/dev/serial/by-id")

# One (PortHandler, sts, Lock) entry per resolved device path so multiple motors
# on the same Waveshare adapter share the serial port safely.
_port_cache: dict[str, tuple[PortHandler, sts, threading.Lock]] = {}
_cache_lock = threading.Lock()


def resolve_serial_device(address: str, by_id_dir: Path = SERIAL_BY_ID_DIR) -> str:
    """Resolve a stored controller address to a serial device path.

    Accepts:
    - an existing device path (``/dev/ttyACM0``, ``/dev/ttyMotor0``, ``/dev/serial/by-id/...``),
    - a raw Waveshare USB serial identifier (e.g. ``5B3D042723``), matched against
      ``/dev/serial/by-id/usb-1a86_USB_Single_Serial_<address>-if00`` or any entry
      in ``/dev/serial/by-id`` containing ``<address>``.

    If no matching symlink exists yet, returns ``address`` unchanged so ``PortHandler``
    reports the clear open failure for that identifier.
    """
    cleaned = address.strip()
    if not cleaned:
        return cleaned

    direct = Path(cleaned)
    if direct.exists():
        return str(direct)

    if by_id_dir.is_dir():
        exact = by_id_dir / f"usb-1a86_USB_Single_Serial_{cleaned}-if00"
        if exact.exists():
            return str(exact)
        for candidate in sorted(by_id_dir.iterdir()):
            if cleaned in candidate.name and candidate.exists():
                return str(candidate)

    return cleaned


def _get_or_open_port(
    address: str, baudrate: int = DEFAULT_BAUDRATE
) -> tuple[PortHandler, sts, threading.Lock]:
    device = resolve_serial_device(address)
    with _cache_lock:
        cached = _port_cache.get(device)
        if cached is not None and cached[0].is_open:
            return cached

        ph = PortHandler(device)
        if not ph.openPort():
            raise RuntimeError(f"Failed to open STS port: {device}")
        if not ph.setBaudRate(baudrate):
            ph.closePort()
            raise RuntimeError(f"Failed to set STS baudrate {baudrate} on {device}")
        pk = sts(ph)
        entry = (ph, pk, threading.Lock())
        _port_cache[device] = entry
        return entry


def centideg_to_ticks(centideg: float) -> int:
    """Map pib centidegrees [-9000..9000] to STS ticks [1000..3000] (center 2000)."""
    if centideg < 0:
        ticks = int(round(1000 + (centideg + 9000) * (1000 / 9000)))
    else:
        ticks = int(round(2000 + centideg * (1050 / 9000)))
    return max(1000, min(3000, ticks))


def ticks_to_centideg(ticks: int) -> int:
    """Map STS ticks [0..4095] back to pib centidegrees."""
    return int(round(float(ticks * 18000 / 4096) - 9000))


class FeetechStSerialActuator:
    """One Feetech STS servo on a Waveshare USB-serial bus."""

    kind: str = "feetech_st_serial"
    CAPABILITIES: frozenset[Any] = frozenset()
    NO_CURRENT: int = -1
    POSITION_TOLERANCE: int = 200

    def __init__(self, pin: int, uid: str, invert: bool) -> None:
        self.pin: int = int(pin)
        self.uid: str = uid
        self.invert: bool = invert
        self._baudrate: int = DEFAULT_BAUDRATE
        self._connected: bool | None = None
        self._target_position: int | None = None
        self._settings: dict[str, Any] = {
            "velocity": DEFAULT_SPEED,
            "acceleration": DEFAULT_ACCEL,
            "deceleration": DEFAULT_ACCEL,
            "pulseWidthMin": 700,
            "pulseWidthMax": 2500,
            "period": 19500,
            "turnedOn": True,
        }
        self._ph: PortHandler | None = None
        self._pk: sts | None = None
        self._lock: threading.Lock | None = None

        self.check_connection()

    def __str__(self) -> str:
        return f"STS-PIN[ id: {self.pin}, device: {self.uid} ]"

    def check_connection(self) -> bool:
        """Open the serial port if needed and probe this servo ID once."""
        try:
            self._ph, self._pk, self._lock = _get_or_open_port(self.uid, self._baudrate)
            with self._lock:
                _, _, res, _err = self._pk.ReadPosSpeed(self.pin)
            self._connected = res == COMM_SUCCESS
        except Exception:
            self._connected = False
        return bool(self._connected)

    def is_connected(self) -> bool:
        # Serial servos only reply once the Solid State Relay powers the motor bus.
        if not self._connected:
            return self.check_connection()
        return bool(self._connected)

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        for key in (
            "velocity",
            "acceleration",
            "deceleration",
            "turnedOn",
            "pulseWidthMin",
            "pulseWidthMax",
            "period",
        ):
            if key in settings_dto:
                self._settings[key] = settings_dto[key]

        if not self.is_connected() or self._pk is None or self._lock is None:
            return False

        if "turnedOn" in settings_dto:
            try:
                enable_val = 1 if settings_dto["turnedOn"] else 0
                with self._lock:
                    self._pk.write1ByteTxRx(self.pin, STS_TORQUE_ENABLE, enable_val)
            except Exception as error:
                logging.error(
                    f"Error while applying STS torque state on {self.pin}: {error}"
                )
                return False
        return True

    def get_settings(self) -> dict[str, Any]:
        return dict(self._settings)

    def get_current(self) -> int:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return self.NO_CURRENT
        try:
            with self._lock:
                sts_present_current, res, _err = self._pk.ReadCurrent(self.pin)
            if res != COMM_SUCCESS:
                return self.NO_CURRENT
            return int(sts_present_current) * 10
        except Exception:
            return self.NO_CURRENT

    def set_position(self, position: int) -> bool:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return False

        centideg = -position if self.invert else position
        ticks = centideg_to_ticks(float(centideg))
        if self.pin in STS3095_SERVO_IDS:
            speed, acc = STS3095_SPEED, STS3095_ACCEL
        else:
            speed, acc = FINGER_SPEED, FINGER_ACCEL

        try:
            with self._lock:
                res, err = self._pk.WritePosEx(self.pin, ticks, speed, acc)
            if res != COMM_SUCCESS:
                logging.error(
                    f"STS WritePosEx failed on id {self.pin} (res={res}, err={err})"
                )
                return False
            self._target_position = centideg
            return True
        except Exception as error:
            logging.error(f"Exception during STS WritePosEx: {error}")
            return False

    def has_reached_target(self) -> bool:
        target = self._target_position
        if target is None:
            return True
        current_position = self.get_current_position()
        return abs(current_position - target) <= self.POSITION_TOLERANCE

    def get_position(self) -> int:
        if self._target_position is not None:
            return -self._target_position if self.invert else self._target_position
        return self.get_current_position()

    def get_current_position(self) -> int:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return 0
        try:
            with self._lock:
                ticks, _spd, res, _err = self._pk.ReadPosSpeed(self.pin)
            if res != COMM_SUCCESS:
                return 0
            centideg = ticks_to_centideg(int(ticks))
            return -centideg if self.invert else centideg
        except Exception:
            return 0

    def get_temperature(self) -> int | None:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return None
        try:
            with self._lock:
                temp, res, _err = self._pk.read1ByteTxRx(
                    self.pin, STS_PRESENT_TEMPERATURE
                )
            if res != COMM_SUCCESS:
                return None
            return int(temp)
        except Exception:
            return None
