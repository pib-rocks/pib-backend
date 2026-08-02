"""Unit tests for periodic motor-current telemetry (PR-1510)."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch


MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "ros_packages"
    / "motors"
    / "motors"
    / "motor_current.py"
)


def _module(name, **attributes):
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


def _load_motor_current(motors):
    class FakeDiagnosticStatus:
        OK = 0
        WARN = 1

    class FakeKeyValue:
        pass

    class FakeMotor:
        NO_CURRENT = -1

    class FakePublisher:
        def __init__(self):
            self.publish = MagicMock()

    class FakeNode:
        def __init__(self, name):
            self.name = name
            self.created_timers = []
            self._logger = MagicMock()

        def create_publisher(self, message_type, topic, qos):
            self.publisher_arguments = (message_type, topic, qos)
            return FakePublisher()

        def create_timer(self, period, callback):
            timer = types.SimpleNamespace(period=period, callback=callback)
            self.created_timers.append(timer)
            return timer

        def get_logger(self):
            return self._logger

    class FakeBrickletServoV2:
        CALLBACK_SERVO_CURRENT = 27

    rclpy = _module("rclpy")
    rclpy_node = _module("rclpy.node", Node=FakeNode)
    rclpy.node = rclpy_node

    diagnostic_msgs = _module("diagnostic_msgs")
    diagnostic_msgs_msg = _module(
        "diagnostic_msgs.msg",
        DiagnosticStatus=FakeDiagnosticStatus,
        KeyValue=FakeKeyValue,
    )
    diagnostic_msgs.msg = diagnostic_msgs_msg

    pib_motors = _module("pib_motors")
    pib_motors_motor = _module("pib_motors.motor", motors=motors, Motor=FakeMotor)
    pib_motors.motor = pib_motors_motor

    tinkerforge = _module("tinkerforge")
    servo_v2 = _module(
        "tinkerforge.bricklet_servo_v2", BrickletServoV2=FakeBrickletServoV2
    )
    tinkerforge.bricklet_servo_v2 = servo_v2

    dependencies = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "diagnostic_msgs": diagnostic_msgs,
        "diagnostic_msgs.msg": diagnostic_msgs_msg,
        "pib_motors": pib_motors,
        "pib_motors.motor": pib_motors_motor,
        "tinkerforge": tinkerforge,
        "tinkerforge.bricklet_servo_v2": servo_v2,
    }

    spec = importlib.util.spec_from_file_location("motor_current_timer_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, dependencies):
        spec.loader.exec_module(module)
    return module


def test_motor_current_creates_one_second_publish_timer():
    module = _load_motor_current([])

    node = module.MotorCurrent()

    assert len(node.created_timers) == 1
    assert node.timer is node.created_timers[0]
    assert node.timer.period == 1.0
    assert node.timer.callback == node.publish_motor_current


def test_publish_motor_current_publishes_diagnostic_status():
    motor = types.SimpleNamespace(
        name="head_motor",
        bricklet_pins=[],
        get_current=MagicMock(return_value=500),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()

    node.publish_motor_current()

    node.motor_current_publisher.publish.assert_called_once()
    message = node.motor_current_publisher.publish.call_args.args[0]
    assert isinstance(message, module.DiagnosticStatus)
    assert message.name == "head_motor"
    assert message.level == module.DiagnosticStatus.OK
    assert message.values[0].key == "head_motor"
    assert message.values[0].value == "500"
