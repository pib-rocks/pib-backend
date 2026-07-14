"""Motor clamping tests — docs/test-basis HC-MOTOR-* constraints."""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

_ip_connection = types.ModuleType("tinkerforge.ip_connection")
_ip_connection.IPConnection = MagicMock()
_ip_connection.Error = Exception
_tinkerforge = types.ModuleType("tinkerforge")
_tinkerforge.brick_hat = MagicMock()
_tinkerforge.bricklet_servo_v2 = MagicMock()
_tinkerforge.bricklet_solid_state_relay_v2 = MagicMock()
_tinkerforge.bricklet_rgb_led_button = MagicMock()
_tinkerforge.ip_connection = _ip_connection
for name, mod in [
    ("tinkerforge", _tinkerforge),
    ("tinkerforge.brick_hat", _tinkerforge.brick_hat),
    ("tinkerforge.bricklet_servo_v2", _tinkerforge.bricklet_servo_v2),
    ("tinkerforge.bricklet_solid_state_relay_v2", _tinkerforge.bricklet_solid_state_relay_v2),
    ("tinkerforge.bricklet_rgb_led_button", _tinkerforge.bricklet_rgb_led_button),
    ("tinkerforge.ip_connection", _ip_connection),
]:
    sys.modules.setdefault(name, mod)

REPO_ROOT = Path(__file__).resolve().parents[2]
for path in (
    str(REPO_ROOT / "ros_packages" / "motors" / "pib_motors"),
    str(REPO_ROOT / "pib_api" / "client"),
):
    if path not in sys.path:
        sys.path.insert(0, path)


@pytest.fixture()
def motor_class():
    for name in list(sys.modules):
        if name.startswith("pib_motors"):
            sys.modules.pop(name, None)
    empty = (True, {"motors": []})
    bricklets = (True, {"bricklets": []})
    with patch("pib_api_client.motor_client.get_all_motors", return_value=empty), patch(
        "pib_api_client.bricklet_client.get_all_bricklets", return_value=bricklets
    ):
        from pib_motors.motor import Motor

        yield Motor


@pytest.fixture()
def connected_motor(motor_class):
    pin = MagicMock()
    pin.apply_settings.return_value = True
    pin.set_position.return_value = True
    pin.get_position.return_value = 0
    pin.get_settings.return_value = {}
    return motor_class("tilt_forward_motor", [pin], invert=False)


class TestMotorPositionClamping:
    @pytest.mark.parametrize(
        "requested,expected",
        [(5000, 4500), (-5000, -4500), (1000, 1000)],
    )
    def test_tilt_forward_clamp(self, connected_motor, requested, expected):
        connected_motor.rotation_range_min = -4500
        connected_motor.rotation_range_max = 4500
        assert connected_motor._validate_position(requested) == expected

    @pytest.mark.parametrize(
        "requested,expected",
        [(12000, 9000), (-12000, -9000)],
    )
    def test_global_envelope(self, motor_class, requested, expected):
        motor = motor_class("turn_head_motor", [], invert=False)
        assert motor._validate_position(requested) == expected

    def test_no_pins_returns_false(self, motor_class):
        assert motor_class("orphan", [], invert=False).set_position(100) is False
