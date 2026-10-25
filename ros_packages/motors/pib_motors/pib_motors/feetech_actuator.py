"""Feetech STS serial bus actuator family (PC-125).

Drives Feetech STS servos attached to Waveshare USB-serial adapters on a
``pib5advanced`` robot without touching ``ServoBrickletActuator`` or the
Tinkerforge stack.
"""

from __future__ import annotations

import fcntl
import logging
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

from pib_motors.stservo_sdk.port_handler import PortHandler
from pib_motors.stservo_sdk.sts import (
    STS_PRESENT_TEMPERATURE,
    STS_PRESENT_VOLTAGE,
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
ZERO_CALIBRATION_SETTLE_SEC = 0.02

# STS position encoding: 4096 ticks per full revolution; joint zero aligns with tick 2048.
TICKS_PER_REVOLUTION = 4096
TICKS_AT_JOINT_ZERO = 2048
CENTIDEG_PER_REVOLUTION = 36000

_DEFAULT_ROTATION_RANGE_MIN = -9000
_DEFAULT_ROTATION_RANGE_MAX = 9000

SERIAL_BY_ID_DIR = Path("/dev/serial/by-id")

# One (PortHandler, sts, Lock) entry per resolved device path so multiple motors
# on the same Waveshare adapter share the serial port safely.
_port_cache: dict[str, tuple[PortHandler, sts, threading.Lock]] = {}
_cache_lock = threading.Lock()


class _BusAccess:
    """Serializes bus transactions across threads and OS processes (motor_control + motor_current)."""

    def __init__(self, ph: PortHandler, lock: threading.Lock) -> None:
        self._ph = ph
        self._lock = lock

    def __enter__(self) -> None:
        self._lock.acquire()
        ser = getattr(self._ph, "ser", None)
        if ser is not None and hasattr(ser, "fileno"):
            try:
                fcntl.flock(ser.fileno(), fcntl.LOCK_EX)
            except Exception:
                pass

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        ser = getattr(self._ph, "ser", None)
        if ser is not None and hasattr(ser, "fileno"):
            try:
                fcntl.flock(ser.fileno(), fcntl.LOCK_UN)
            except Exception:
                pass
        self._lock.release()


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


def centideg_to_ticks(
    centideg: float,
    rotation_range_min: int | None = None,
    rotation_range_max: int | None = None,
) -> int:
    """Map pib centidegrees to STS position ticks.

    Geometry: ``TICKS_PER_REVOLUTION`` (4096) ticks per revolution; joint zero is
    ``TICKS_AT_JOINT_ZERO`` (2048). Conversion uses ``round()`` toward the nearest
    integer tick. Values outside the allowed centidegree range raise ``ValueError``
    instead of clamping.

    When ``rotation_range_min`` / ``rotation_range_max`` are omitted, the legal
    range is ±90° (±9000 centidegrees).
    """
    range_min = (
        rotation_range_min
        if rotation_range_min is not None
        else _DEFAULT_ROTATION_RANGE_MIN
    )
    range_max = (
        rotation_range_max
        if rotation_range_max is not None
        else _DEFAULT_ROTATION_RANGE_MAX
    )
    if centideg < range_min or centideg > range_max:
        raise ValueError(
            f"centidegree {centideg} outside allowed range [{range_min}, {range_max}]"
        )
    return int(
        round(
            TICKS_AT_JOINT_ZERO
            + centideg * TICKS_PER_REVOLUTION / CENTIDEG_PER_REVOLUTION
        )
    )


def ticks_to_centideg(ticks: int) -> int:
    """Map raw STS position ticks back to pib centidegrees.

    Uses the same 4096-ticks-per-revolution geometry with zero at tick 2048 as
    :func:`centideg_to_ticks` (exact inverse aside from rounding). Raw ticks below 0
    or above 4095 are converted without error; callers decide if the value is valid.

    Reported actual positions from hardware change when this mapping is corrected
    (previously roughly half-scale with an offset).
    """
    return int(
        round(
            (ticks - TICKS_AT_JOINT_ZERO)
            * CENTIDEG_PER_REVOLUTION
            / TICKS_PER_REVOLUTION
        )
    )


class FeetechStSerialActuator:
    """One Feetech STS servo on a Waveshare USB-serial bus."""

    kind: str = "feetech_st_serial"
    CAPABILITIES: frozenset[Any] = frozenset()
    NO_CURRENT: int = -1
    POSITION_TOLERANCE: int = 200

    def __init__(self, pin: int, uid: str, upper_endpoint: str) -> None:
        self.pin: int = int(pin)
        self.uid: str = uid
        self.upper_endpoint: str = upper_endpoint
        self.direction_sign: int = -1 if upper_endpoint == "min" else 1
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

    @contextmanager
    def _bus(self) -> Iterator[None]:
        assert self._ph is not None and self._lock is not None
        with _BusAccess(self._ph, self._lock):
            yield

    def check_connection(self) -> bool:
        """Open the serial port if needed and probe this servo ID once."""
        try:
            self._ph, self._pk, self._lock = _get_or_open_port(self.uid, self._baudrate)
            with self._bus():
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

    def _resolve_speed_and_acc(self) -> tuple[int, int]:
        if self.pin in STS3095_SERVO_IDS:
            default_speed, default_acc = STS3095_SPEED, STS3095_ACCEL
        else:
            default_speed, default_acc = FINGER_SPEED, FINGER_ACCEL

        velocity = self._settings.get("velocity")
        if velocity is None or velocity <= 0:
            speed = default_speed
        elif velocity in (DEFAULT_SPEED, 10000, 16000, 100000):
            speed = default_speed
        elif 1 <= velocity <= 3400:
            speed = int(velocity)
        else:
            speed = max(50, min(3400, int(round(velocity * 3400 / 500000))))

        acceleration = self._settings.get("acceleration")
        if acceleration in (None, 0, DEFAULT_ACCEL, 10000, 50000):
            acc = default_acc
        elif 1 <= acceleration <= 254:
            acc = int(acceleration)
        else:
            acc = max(1, min(254, int(round(acceleration * 254 / 500000))))

        return speed, acc

    def calibrate_zero(self) -> bool:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return False
        speed, acc = self._resolve_speed_and_acc()
        try:
            with self._bus():
                res, _err = self._pk.write1ByteTxRx(self.pin, STS_TORQUE_ENABLE, 128)
            if res != COMM_SUCCESS:
                return False
            if ZERO_CALIBRATION_SETTLE_SEC > 0:
                time.sleep(ZERO_CALIBRATION_SETTLE_SEC)
            with self._bus():
                res_pos, _err_pos = self._pk.WritePosEx(
                    self.pin, TICKS_AT_JOINT_ZERO, speed, acc
                )
                res_en, _err_en = self._pk.write1ByteTxRx(
                    self.pin, STS_TORQUE_ENABLE, 1
                )
            if res_pos != COMM_SUCCESS or res_en != COMM_SUCCESS:
                return False
            self._settings["turnedOn"] = True
            self._target_position = 0
            return True
        except Exception as error:
            logging.error(f"Error during STS zero calibration on {self.pin}: {error}")
            return False

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        if settings_dto.get("calibrateZero") is True:
            return self.calibrate_zero()

        upper_endpoint = settings_dto.get(
            "upperEndpoint", settings_dto.get("upper_endpoint")
        )
        if upper_endpoint is not None:
            if upper_endpoint not in ("min", "max"):
                raise ValueError("upper_endpoint must be 'min' or 'max'")
            self.upper_endpoint = upper_endpoint
            self.direction_sign = -1 if upper_endpoint == "min" else 1

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
                with self._bus():
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
            with self._bus():
                sts_present_current, res, _err = self._pk.ReadCurrent(self.pin)
            if res != COMM_SUCCESS:
                return self.NO_CURRENT
            return int(sts_present_current) * 10
        except Exception:
            return self.NO_CURRENT

    def set_position(self, position: int) -> bool:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return False

        position = int(round(position))
        centideg = position * self.direction_sign
        ticks = centideg_to_ticks(float(centideg))
        speed, acc = self._resolve_speed_and_acc()

        try:
            with self._bus():
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
            return int(round(self._target_position * self.direction_sign))
        return self.get_current_position()

    def get_current_position(self) -> int:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return 0
        try:
            with self._bus():
                ticks, _spd, res, _err = self._pk.ReadPosSpeed(self.pin)
            if res != COMM_SUCCESS:
                return 0
            centideg = ticks_to_centideg(int(ticks))
            return int(round(centideg * self.direction_sign))
        except Exception:
            return 0

    def get_temperature(self) -> int | None:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return None
        try:
            with self._bus():
                temp, res, _err = self._pk.read1ByteTxRx(
                    self.pin, STS_PRESENT_TEMPERATURE
                )
            if res != COMM_SUCCESS:
                return None
            return int(temp)
        except Exception:
            return None

    def get_voltage(self) -> float | None:
        if not self.is_connected() or self._pk is None or self._lock is None:
            return None
        try:
            with self._bus():
                raw_v, res, _err = self._pk.read1ByteTxRx(self.pin, STS_PRESENT_VOLTAGE)
            if res != COMM_SUCCESS:
                return None
            return round(int(raw_v) / 10.0, 2)
        except Exception:
            return None
