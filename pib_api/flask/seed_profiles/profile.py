"""Data structures for hardware-specific database seed profiles."""

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class ControllerProfile:
    number: int
    kind: str
    #: The Tinkerforge bricklet type, or None for every other controller kind -
    #: ``Controller`` rejects a device type on a serial or CAN controller.
    device_type: str | None
    supply_voltage: float | None
    #: Bricklet UID or serial interface; None means "not read and entered yet".
    address: str | None = None


@dataclass(frozen=True)
class HardwareProfile:
    variant: str
    description: str
    controllers: tuple[ControllerProfile, ...]
    motor_mapping: Mapping[str, tuple[int, int]]
    rgb_button_controller_ids: tuple[int, int, int]
    motor_parameter_defaults: Mapping[str, Any]
    motor_parameter_deviations: Mapping[str, Mapping[str, Any]]
    microphone_tuning: Mapping[str, Any]
