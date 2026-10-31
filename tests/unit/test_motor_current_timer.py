"""Unit tests for periodic motor-current telemetry (PR-1510)."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch

PORT_PROBE_PATH = (
    Path(__file__).resolve().parents[2]
    / "ros_packages"
    / "motors"
    / "pib_motors"
    / "pib_motors"
    / "port_probe.py"
)

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


def _load_motor_current(motors, relay_state=None, probe=None):
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

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscriptions = getattr(self, "subscriptions", [])
            subscription = types.SimpleNamespace(
                message_type=message_type, topic=topic, callback=callback
            )
            self.subscriptions.append(subscription)
            return subscription

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

    class FakeRelayBricklet:
        def __init__(self, state):
            self._state = state

        def get_state(self):
            return self._state

    class FakeSolidStateRelayState:
        pass

    datatypes = _module("datatypes")
    datatypes_msg = _module(
        "datatypes.msg", SolidStateRelayState=FakeSolidStateRelayState
    )
    datatypes.msg = datatypes_msg

    pib_motors_bricklet = _module(
        "pib_motors.bricklet",
        solid_state_relay_bricklet=(
            None if relay_state is None else FakeRelayBricklet(relay_state)
        ),
    )
    pib_motors.port_probe = probe

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
        "pib_motors.bricklet": pib_motors_bricklet,
        "pib_motors.port_probe": probe
        or _module(
            "pib_motors.port_probe",
            port_answers=lambda actuator: True,
            port_key=lambda actuator: str(getattr(actuator, "uid", "")),
        ),
        "datatypes": datatypes,
        "datatypes.msg": datatypes_msg,
        "tinkerforge": tinkerforge,
        "tinkerforge.bricklet_servo_v2": servo_v2,
    }

    spec = importlib.util.spec_from_file_location(
        "motor_current_timer_under_test", MODULE_PATH
    )
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


def test_setup_callbacks_skips_actuators_without_a_bricklet():
    class BrickletLessActuator:
        """An actuator of a non-Tinkerforge family: it has no 'bricklet' attribute."""

        def is_connected(self):
            return True

    bricklet = MagicMock()
    servo_actuator = types.SimpleNamespace(
        bricklet=bricklet, pin=3, is_connected=lambda: True
    )
    motor = types.SimpleNamespace(
        name="shoulder_vertical_left",
        actuators=[BrickletLessActuator(), servo_actuator],
    )
    module = _load_motor_current([motor])

    node = module.MotorCurrent()  # registers the callbacks for all motors

    # only the servo actuator is registered, the other family is silently skipped
    assert node.pin_to_motors == {(id(bricklet), 3): ["shoulder_vertical_left"]}
    bricklet.register_callback.assert_called_once()


def test_publish_motor_current_publishes_diagnostic_status():
    motor = types.SimpleNamespace(
        name="head_motor",
        actuators=[],
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


def test_publish_motor_current_includes_actual_position_and_temperature_for_serial_actuator():
    serial_actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        get_temperature=lambda: 34,
    )
    motor = types.SimpleNamespace(
        name="turn_head_motor",
        actuators=[serial_actuator],
        get_current=MagicMock(return_value=120),
        get_current_position=MagicMock(return_value=-215),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()

    node.publish_motor_current()

    node.motor_current_publisher.publish.assert_called_once()
    message = node.motor_current_publisher.publish.call_args.args[0]
    kv = {entry.key: entry.value for entry in message.values}
    assert kv == {
        "turn_head_motor": "120",
        "actual_position": "-2.1 °",
        "temperature": "34 °C",
    }


def _probe_module(answer, calls):
    def port_answers(actuator):
        calls.append(getattr(actuator, "uid", None))
        return answer

    return _module("pib_motors.port_probe", port_answers=port_answers)


def test_power_off_skips_current_polling_entirely():
    """Load-bearing: 26 reads x 50 ms timeout per tick while the power is off."""

    class TouchyMotor:
        name = "turn_head_motor"
        NO_CURRENT = -1

        def __init__(self):
            self.reads = 0
            self.actuators = []

        def get_current(self):
            self.reads += 1
            return 100

    motor = TouchyMotor()
    module = _load_motor_current([motor], relay_state=False)
    node = module.MotorCurrent()

    node.publish_motor_current()

    assert motor.reads == 0, "keine Messung bei ausgeschalteter Versorgung"
    assert node._motor_power_is_off() is True
    node.get_logger().info.assert_called()


def test_relay_topic_opens_and_closes_the_gate():
    module = _load_motor_current([], relay_state=None)
    node = module.MotorCurrent()

    assert node._motor_power_is_off() is False, "unbekannt zaehlt als an"
    node.on_ssr_state_change(types.SimpleNamespace(turned_on=False))
    assert node._motor_power_is_off() is True
    node.on_ssr_state_change(types.SimpleNamespace(turned_on=True))
    assert node._motor_power_is_off() is False


def test_current_polling_probes_each_port_once():
    """Load-bearing: the real port_probe probes one adapter once, not each servo."""

    class Actuator:
        NO_CURRENT = -1

        def __init__(self, uid, pin):
            self.uid = uid
            self.pin = pin
            self.connected_calls = 0

        def is_connected(self):
            self.connected_calls += 1
            return True

        def get_current(self):
            return 250

        def get_voltage(self):
            return 12.0

    class Motor3:
        name = "arm"
        NO_CURRENT = -1

        def __init__(self, actuators):
            self.actuators = actuators

        def get_current(self):
            return 200

    probe_spec = importlib.util.spec_from_file_location(
        "real_port_probe_under_test", PORT_PROBE_PATH
    )
    real_probe = importlib.util.module_from_spec(probe_spec)
    probe_spec.loader.exec_module(real_probe)
    real_probe.forget_dead_ports()

    actuators = [Actuator("adapter-A", pin) for pin in (1, 2, 3)]
    module = _load_motor_current(
        [Motor3(actuators)], relay_state=True, probe=real_probe
    )
    node = module.MotorCurrent()

    node.publish_motor_current()

    assert [a.connected_calls for a in actuators] == [
        1,
        0,
        0,
    ], f"eine Probe pro Port erwartet, war {[a.connected_calls for a in actuators]}"
    real_probe.forget_dead_ports()
