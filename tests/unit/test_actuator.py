"""Unit tests for the actuator abstraction behind a motor (PR-1744)."""

import importlib.util
import sys
import types
from pathlib import Path
from typing import Any
from unittest import mock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
ACTUATOR_MODULE_PATH = PIB_MOTORS_DIR / "pib_motors" / "actuator.py"
MOTOR_MODULE_PATH = PIB_MOTORS_DIR / "pib_motors" / "motor.py"

for path in (
    str(REPO_ROOT / "pib_api" / "client"),
    str(PIB_MOTORS_DIR),
):
    if path not in sys.path:
        sys.path.insert(0, path)

# Imported here so that the module motor.py loads its motors from stays the same
# object across the sys.modules patching done while importing it.
from pib_api_client import motor_client  # noqa: E402

SERVO_UID = "SERVO1"

SETTINGS_DTO = {
    "visible": True,
    "invert": False,
    "rotationRangeMin": -9000,
    "rotationRangeMax": 9000,
    "pulseWidthMin": 800,
    "pulseWidthMax": 2200,
    "velocity": 16000,
    "acceleration": 3000,
    "deceleration": 5000,
    "period": 19500,
    "turnedOn": True,
}


class FakeActuator:
    """Records what a motor commands onto one axis."""

    def __init__(
        self, current: int = 0, reached: bool = True, connected: bool = True
    ) -> None:
        self.commanded_positions: list[int] = []
        self.applied_settings: list[dict[str, Any]] = []
        self._current = current
        self._reached = reached
        self._connected = connected

    def set_position(self, position: int) -> bool:
        self.commanded_positions.append(position)
        return True

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        self.applied_settings.append(settings_dto)
        return True

    def get_settings(self) -> dict[str, Any]:
        return {}

    def get_current(self) -> int:
        return self._current

    def has_reached_target(self) -> bool:
        return self._reached

    def is_connected(self) -> bool:
        return self._connected

    def get_position(self) -> int:
        return 0

    def get_current_position(self) -> int:
        return 0


class FakeServoBricklet:
    """Stands in for a BrickletServoV2, which needs a running brickd."""

    def __init__(self, current: int = 0, current_position: int = 0) -> None:
        self.current = current
        self.current_position = current_position
        self.calls: list[tuple] = []
        self.commanded_positions: list[tuple[int, int]] = []

    def get_servo_current(self, pin):
        return self.current

    def get_current_position(self, pin):
        return self.current_position

    def set_pulse_width(self, pin, pulse_width_min, pulse_width_max):
        self.calls.append(("set_pulse_width", pin, pulse_width_min, pulse_width_max))

    def set_motion_configuration(self, pin, velocity, acceleration, deceleration):
        self.calls.append(
            ("set_motion_configuration", pin, velocity, acceleration, deceleration)
        )

    def set_period(self, pin, period):
        self.calls.append(("set_period", pin, period))

    def set_enable(self, pin, turned_on):
        self.calls.append(("set_enable", pin, turned_on))

    def set_position(self, pin, position):
        self.commanded_positions.append((pin, position))


def _module(name: str, **attributes) -> types.ModuleType:
    fake = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(fake, key, value)
    return fake


def _fake_dependencies(uid_to_servo_bricklet: dict) -> dict[str, types.ModuleType]:
    """Stand in for the tinkerforge stack and the bricklets loaded from pib-api."""

    class FakeError(Exception):
        def __init__(self, value=0, description=""):
            super().__init__(description)
            self.value = value

    return {
        "tinkerforge": _module("tinkerforge"),
        "tinkerforge.bricklet_servo_v2": _module(
            "tinkerforge.bricklet_servo_v2", BrickletServoV2=FakeServoBricklet
        ),
        "tinkerforge.ip_connection": _module(
            "tinkerforge.ip_connection", Error=FakeError
        ),
        "pib_motors.bricklet": _module(
            "pib_motors.bricklet", uid_to_servo_bricklet=uid_to_servo_bricklet
        ),
    }


def _load_actuator_module(uid_to_servo_bricklet: dict | None = None):
    """Import actuator.py the way the motor nodes do, without brickd."""
    spec = importlib.util.spec_from_file_location(
        "actuator_under_test", ACTUATOR_MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)

    with mock.patch.dict(sys.modules, _fake_dependencies(uid_to_servo_bricklet or {})):
        spec.loader.exec_module(module)

    return module


def _load_motor_module(actuator_module):
    """Import motor.py against the given actuator module, with no motors configured."""
    spec = importlib.util.spec_from_file_location("motor_under_test", MOTOR_MODULE_PATH)
    module = importlib.util.module_from_spec(spec)

    dependencies = _fake_dependencies({})
    dependencies["pib_motors.actuator"] = actuator_module

    with (
        mock.patch.dict(sys.modules, dependencies),
        mock.patch.object(
            motor_client, "get_all_motors", return_value=(True, {"motors": []})
        ),
    ):
        spec.loader.exec_module(module)

    return module


@pytest.fixture()
def motor_module():
    return _load_motor_module(_load_actuator_module())


def test_motor_clamps_a_commanded_position_to_its_rotation_range(motor_module):
    actuator = FakeActuator()
    motor = motor_module.Motor("head", [actuator], invert=False)
    motor.rotation_range_min = -1000
    motor.rotation_range_max = 1000

    assert motor.set_position(5000) is True
    assert motor.set_position(-5000) is True
    assert motor.set_position(500) is True

    assert actuator.commanded_positions == [1000, -1000, 500]


def test_motor_applies_invert_exactly_once(motor_module):
    inverted = FakeActuator()
    plain = FakeActuator()

    motor_module.Motor("head", [inverted], invert=True).set_position(1000)
    motor_module.Motor("head", [plain], invert=False).set_position(1000)

    # Inverting twice would land on +1000 again, just like not inverting at all.
    assert inverted.commanded_positions == [-1000]
    assert plain.commanded_positions == [1000]


def test_motor_current_is_the_maximum_across_its_actuators(motor_module):
    actuators = [
        FakeActuator(current=100),
        FakeActuator(current=700),
        FakeActuator(current=300),
    ]

    motor = motor_module.Motor("head", actuators, invert=False)

    assert motor.get_current() == 700


def test_motor_without_actuators_reports_no_current(motor_module):
    motor = motor_module.Motor("head", [], invert=False)

    assert motor.get_current() == motor_module.Motor.NO_CURRENT == -1


def test_motor_has_reached_position_only_when_every_actuator_has(motor_module):
    all_reached = [FakeActuator(reached=True), FakeActuator(reached=True)]
    one_short = [FakeActuator(reached=True), FakeActuator(reached=False)]

    assert motor_module.Motor("head", all_reached, False).has_reached_position() is True
    assert motor_module.Motor("head", one_short, False).has_reached_position() is False


def test_motor_forwards_settings_to_every_actuator(motor_module):
    actuators = [FakeActuator(), FakeActuator()]
    motor = motor_module.Motor("head", actuators, invert=False)

    assert motor.apply_settings(SETTINGS_DTO) is True

    for actuator in actuators:
        assert actuator.applied_settings == [SETTINGS_DTO]
    assert motor.rotation_range_min == -9000
    assert motor.rotation_range_max == 9000


def test_bricklet_pins_is_the_same_list_as_actuators(motor_module):
    actuators = [FakeActuator()]

    motor = motor_module.Motor("head", actuators, invert=False)

    assert motor.bricklet_pins is motor.actuators


def test_fake_actuator_satisfies_the_actuator_protocol():
    actuator_module = _load_actuator_module()

    assert isinstance(FakeActuator(), actuator_module.Actuator)


def test_create_actuator_builds_a_tinkerforge_servo():
    bricklet = FakeServoBricklet()
    actuator_module = _load_actuator_module({SERVO_UID: bricklet})

    actuator = actuator_module.create_actuator("tinkerforge_servo", 3, SERVO_UID, False)

    assert isinstance(actuator, actuator_module.ServoBrickletActuator)
    assert isinstance(actuator, actuator_module.Actuator)
    assert actuator.kind == "tinkerforge_servo"
    assert actuator.pin == 3
    assert actuator.bricklet is bricklet


def test_create_actuator_rejects_an_unknown_kind():
    actuator_module = _load_actuator_module()

    with pytest.raises(ValueError) as raised:
        actuator_module.create_actuator("dynamixel_xl330", 3, SERVO_UID, False)

    assert "dynamixel_xl330" in str(raised.value)
    assert "tinkerforge_servo" in str(raised.value)


def test_capabilities_of_the_tinkerforge_servo():
    actuator_module = _load_actuator_module()
    capability = actuator_module.Capability

    assert actuator_module.capabilities_for("tinkerforge_servo") == frozenset(
        {capability.CURRENT, capability.TARGET_POSITION}
    )


def test_capabilities_for_rejects_an_unknown_kind():
    actuator_module = _load_actuator_module()

    with pytest.raises(ValueError) as raised:
        actuator_module.capabilities_for("dynamixel_xl330")

    assert "dynamixel_xl330" in str(raised.value)


def test_servo_actuator_maps_the_settings_onto_the_bricklet():
    bricklet = FakeServoBricklet()
    actuator_module = _load_actuator_module({SERVO_UID: bricklet})
    actuator = actuator_module.create_actuator("tinkerforge_servo", 4, SERVO_UID, False)

    assert actuator.apply_settings(SETTINGS_DTO) is True

    assert bricklet.calls == [
        ("set_pulse_width", 4, 800, 2200),
        ("set_motion_configuration", 4, 16000, 3000, 5000),
        ("set_period", 4, 19500),
        ("set_enable", 4, True),
    ]


def test_servo_actuator_negates_the_position_when_inverted():
    bricklet = FakeServoBricklet()
    actuator_module = _load_actuator_module({SERVO_UID: bricklet})

    inverted = actuator_module.create_actuator("tinkerforge_servo", 4, SERVO_UID, True)
    plain = actuator_module.create_actuator("tinkerforge_servo", 5, SERVO_UID, False)

    assert inverted.set_position(1000) is True
    assert plain.set_position(1000) is True

    assert bricklet.commanded_positions == [(4, -1000), (5, 1000)]


def test_servo_actuator_reaches_its_target_within_the_position_tolerance():
    bricklet = FakeServoBricklet(current_position=1015)
    actuator_module = _load_actuator_module({SERVO_UID: bricklet})
    actuator = actuator_module.create_actuator("tinkerforge_servo", 4, SERVO_UID, False)

    assert actuator_module.ServoBrickletActuator.POSITION_TOLERANCE == 20
    # Never commanded to move, so there is no target it could be short of.
    assert actuator.has_reached_target() is True

    actuator.set_position(1000)
    assert actuator.has_reached_target() is True

    actuator.set_position(900)
    assert actuator.has_reached_target() is False


def test_servo_actuator_without_a_bricklet_is_not_connected():
    actuator_module = _load_actuator_module({})

    actuator = actuator_module.create_actuator("tinkerforge_servo", 4, "MISSING", False)

    assert actuator.is_connected() is False
    assert actuator.get_current() == actuator_module.ServoBrickletActuator.NO_CURRENT
    assert actuator.set_position(1000) is False
    assert actuator.apply_settings(SETTINGS_DTO) is False
    assert actuator.get_settings() == {}
    assert actuator.get_position() == 0
    assert actuator.get_current_position() == 0
