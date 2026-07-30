"""Unit tests for motor_current.py callback refactoring (PR-1502)."""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch, call

import pytest

# Ensure rclpy and diagnostic_msgs are mocked if not installed in host environment
if "rclpy" not in sys.modules:
    mock_rclpy = types.ModuleType("rclpy")
    mock_node_mod = types.ModuleType("rclpy.node")

    class FakeNode:
        def __init__(self, name):
            self.name = name
            self._logger = MagicMock()
            self.motor_current_publisher = MagicMock()

        def get_logger(self):
            return self._logger

        def create_publisher(self, msg_type, topic, qos):
            return self.motor_current_publisher

    mock_node_mod.Node = FakeNode
    mock_rclpy.node = mock_node_mod
    sys.modules["rclpy"] = mock_rclpy
    sys.modules["rclpy.node"] = mock_node_mod

if "diagnostic_msgs" not in sys.modules or "diagnostic_msgs.msg" not in sys.modules:
    mock_diag_msgs = types.ModuleType("diagnostic_msgs")
    mock_diag_msg_sub = types.ModuleType("diagnostic_msgs.msg")

    class FakeDiagnosticStatus:
        OK = 0
        WARN = 1

    class FakeKeyValue:
        def __init__(self):
            self.key = ""
            self.value = ""

    mock_diag_msg_sub.DiagnosticStatus = FakeDiagnosticStatus
    mock_diag_msg_sub.KeyValue = FakeKeyValue
    mock_diag_msgs.msg = mock_diag_msg_sub
    sys.modules["diagnostic_msgs"] = mock_diag_msgs
    sys.modules["diagnostic_msgs.msg"] = mock_diag_msg_sub

# Ensure tinkerforge modules are mocked consistently
_ip_connection = types.ModuleType("tinkerforge.ip_connection")
_ip_connection.IPConnection = MagicMock()
_ip_connection.Error = Exception

_servo_v2 = types.ModuleType("tinkerforge.bricklet_servo_v2")


class FakeBrickletServoV2:
    CALLBACK_SERVO_CURRENT = 27

    def set_servo_current_configuration(
        self, servo_channel, averaging_duration, value_has_to_change=False
    ):
        pass

    def register_callback(self, callback_id, function):
        pass


_servo_v2.BrickletServoV2 = FakeBrickletServoV2

_tinkerforge = types.ModuleType("tinkerforge")
_tinkerforge.brick_hat = MagicMock()
_tinkerforge.bricklet_servo_v2 = _servo_v2
_tinkerforge.bricklet_solid_state_relay_v2 = MagicMock()
_tinkerforge.bricklet_rgb_led_button = MagicMock()
_tinkerforge.ip_connection = _ip_connection

for name, mod in [
    ("tinkerforge", _tinkerforge),
    ("tinkerforge.brick_hat", _tinkerforge.brick_hat),
    ("tinkerforge.bricklet_servo_v2", _servo_v2),
    ("tinkerforge.bricklet_solid_state_relay_v2", _tinkerforge.bricklet_solid_state_relay_v2),
    ("tinkerforge.bricklet_rgb_led_button", _tinkerforge.bricklet_rgb_led_button),
    ("tinkerforge.ip_connection", _ip_connection),
]:
    sys.modules[name] = mod

REPO_ROOT = Path(__file__).resolve().parents[2]
for p in (
    str(REPO_ROOT / "ros_packages" / "motors"),
    str(REPO_ROOT / "ros_packages" / "motors" / "pib_motors"),
    str(REPO_ROOT / "pib_api" / "client"),
):
    if p not in sys.path:
        sys.path.insert(0, p)


@pytest.fixture()
def mock_motor_setup():
    """Mock motor and bricklet setup for MotorCurrent testing."""
    for name in list(sys.modules):
        if name.startswith("pib_motors") or name.startswith("motors"):
            sys.modules.pop(name, None)

    mock_bricklet = MagicMock()
    mock_bricklet.set_servo_current_configuration = MagicMock()
    mock_bricklet.register_callback = MagicMock()

    mock_pin0 = MagicMock()
    mock_pin0.is_connected.return_value = True
    mock_pin0.bricklet = mock_bricklet
    mock_pin0.pin = 0

    mock_pin1 = MagicMock()
    mock_pin1.is_connected.return_value = True
    mock_pin1.bricklet = mock_bricklet
    mock_pin1.pin = 1

    motor1 = MagicMock()
    motor1.name = "head_motor"
    motor1.bricklet_pins = [mock_pin0]

    motor2 = MagicMock()
    motor2.name = "arm_motor"
    motor2.bricklet_pins = [mock_pin1]

    mock_motors = [motor1, motor2]

    empty = (True, {"motors": []})
    bricklets = (True, {"bricklets": []})

    with patch(
        "pib_api_client.motor_client.get_all_motors", return_value=empty
    ), patch(
        "pib_api_client.bricklet_client.get_all_bricklets",
        return_value=bricklets,
    ):
        from motors.motor_current import MotorCurrent

        node = MotorCurrent()
        node.setup_callbacks(mock_motors)
        yield node, mock_bricklet, mock_motors


def test_motor_current_initialization_configures_callbacks(mock_motor_setup):
    node, mock_bricklet, _ = mock_motor_setup

    # Verify set_servo_current_configuration called with value_has_to_change=True for both pins
    mock_bricklet.set_servo_current_configuration.assert_has_calls(
        [
            call(0, 100, value_has_to_change=True),
            call(1, 100, value_has_to_change=True),
        ],
        any_order=True,
    )

    # Verify callback registered with CALLBACK_SERVO_CURRENT
    assert mock_bricklet.register_callback.called
    cb_id, callback_func = mock_bricklet.register_callback.call_args[0]
    assert cb_id == 27
    assert callable(callback_func)


def test_motor_current_callback_publishes_diagnostic_status(
    mock_motor_setup,
):
    node, mock_bricklet, _ = mock_motor_setup

    # Retrieve registered callback function
    _, callback_func = mock_bricklet.register_callback.call_args[0]

    # Trigger callback for pin 0 (head_motor) with normal current
    callback_func(0, 500)
    assert node.motor_current_publisher.publish.called
    published_msg = node.motor_current_publisher.publish.call_args[0][0]
    assert published_msg.name == "head_motor"
    assert published_msg.level == 0  # OK
    assert published_msg.values[0].value == "500"

    # Reset publisher mock
    node.motor_current_publisher.publish.reset_mock()

    # Trigger callback for pin 1 (arm_motor) with high current (>= 1500 -> WARN)
    callback_func(1, 1800)
    assert node.motor_current_publisher.publish.called
    published_msg = node.motor_current_publisher.publish.call_args[0][0]
    assert published_msg.name == "arm_motor"
    assert published_msg.level == 1  # WARN
    assert published_msg.values[0].value == "1800"


def test_motor_current_ignores_no_current(mock_motor_setup):
    node, mock_bricklet, _ = mock_motor_setup

    _, callback_func = mock_bricklet.register_callback.call_args[0]

    node.motor_current_publisher.publish.reset_mock()
    # Trigger callback with NO_CURRENT (-1)
    callback_func(0, -1)
    assert not node.motor_current_publisher.publish.called
