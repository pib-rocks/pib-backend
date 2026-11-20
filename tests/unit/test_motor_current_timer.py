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


def test_motor_current_creates_half_second_publish_timer():
    module = _load_motor_current([])

    node = module.MotorCurrent()

    assert len(node.created_timers) == 1
    assert node.timer is node.created_timers[0]
    assert node.timer.period == 0.5
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
    voltage = MagicMock(return_value=11.8)
    serial_actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        get_temperature=lambda: 34,
        get_voltage=voltage,
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
    node.publish_motor_current()

    assert motor.get_current_position.call_count == 1
    assert voltage.call_count == 1
    assert node.motor_current_publisher.publish.call_count == 2
    first_message = node.motor_current_publisher.publish.call_args_list[0].args[0]
    first_kv = {entry.key: entry.value for entry in first_message.values}
    assert first_kv == {
        "turn_head_motor": "120",
        "temperature": "34 °C",
    }
    second_message = node.motor_current_publisher.publish.call_args_list[1].args[0]
    second_kv = {entry.key: entry.value for entry in second_message.values}
    assert second_kv == {
        "turn_head_motor": "120",
        "actual_position": "-2.1 °",
        "temperature": "34 °C",
        "voltage": "11.8 V",
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

    # The diagnostics batch runs on the slow tick (every second call), so the probe
    # happens there - the fast tick must not probe at all.
    node.publish_motor_current()
    assert [a.connected_calls for a in actuators] == [0, 0, 0]
    node.publish_motor_current()

    assert [a.connected_calls for a in actuators] == [
        1,
        0,
        0,
    ], f"eine Probe pro Port erwartet, war {[a.connected_calls for a in actuators]}"
    real_probe.forget_dead_ports()


def test_fast_tick_reads_current_and_temperature_not_position_or_voltage():
    temp_reads = 0
    voltage_reads = 0

    def read_temperature():
        nonlocal temp_reads
        temp_reads += 1
        return 50

    def read_voltage():
        nonlocal voltage_reads
        voltage_reads += 1
        return 12.0

    serial_actuator = types.SimpleNamespace(
        get_temperature=read_temperature,
        get_voltage=read_voltage,
    )
    motor = types.SimpleNamespace(
        name="shoulder_vertical_right",
        actuators=[serial_actuator],
        get_current=MagicMock(return_value=280),
        get_current_position=MagicMock(return_value=0),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()

    node.publish_motor_current()

    motor.get_current.assert_called_once()
    assert temp_reads == 1
    assert motor.get_current_position.call_count == 0
    assert voltage_reads == 0
    message = node.motor_current_publisher.publish.call_args.args[0]
    keys = {entry.key for entry in message.values}
    assert keys == {"shoulder_vertical_right", "temperature"}


def test_slow_tick_repeats_cached_position_and_voltage_on_following_fast_tick():
    serial_actuator = types.SimpleNamespace(
        get_temperature=lambda: 66,
        get_voltage=MagicMock(return_value=12.1),
    )
    position = MagicMock(side_effect=[100, 200])
    motor = types.SimpleNamespace(
        name="shoulder_vertical_right",
        actuators=[serial_actuator],
        get_current=MagicMock(return_value=300),
        get_current_position=position,
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()

    node.publish_motor_current()
    node.publish_motor_current()
    node.publish_motor_current()

    assert position.call_count == 1
    assert serial_actuator.get_voltage.call_count == 1
    third_message = node.motor_current_publisher.publish.call_args_list[2].args[0]
    third_kv = {entry.key: entry.value for entry in third_message.values}
    assert third_kv == {
        "shoulder_vertical_right": "300",
        "temperature": "66 °C",
        "actual_position": "1.0 °",
        "voltage": "12.1 V",
    }


def test_diagnostics_batch_runs_on_the_slow_tick_only():
    """The hardware-page batch reads voltage per actuator, so it keeps its 1 Hz cadence."""
    voltage_reads = 0

    def read_voltage():
        nonlocal voltage_reads
        voltage_reads += 1
        return 12.2

    serial_actuator = types.SimpleNamespace(
        uid="5B79018060",
        pin=3,
        bricklet=None,
        get_temperature=lambda: 40,
        get_voltage=read_voltage,
        get_current=MagicMock(return_value=120),
        NO_CURRENT=-1,
    )
    motor = types.SimpleNamespace(
        name="elbow_left",
        actuators=[serial_actuator],
        get_current=MagicMock(return_value=120),
        get_current_position=MagicMock(return_value=0),
    )
    module = _load_motor_current([motor])
    module.motor_client = MagicMock()
    node = module.MotorCurrent()

    node.publish_motor_current()
    module.motor_client.report_diagnostics_telemetry.assert_not_called()
    assert voltage_reads == 0

    node.publish_motor_current()
    assert module.motor_client.report_diagnostics_telemetry.call_count == 1
    # Single-actuator motors reuse the voltage read from row extras for the
    # diagnostics batch on the slow tick instead of issuing a duplicate bus read.
    assert voltage_reads == 1


def test_bus_timeout_of_one_motor_does_not_stop_the_tick():
    """A Tinkerforge TIMEOUT used to escape the timer callback and kill the node."""

    class BusTimeout(Exception):
        pass

    failing = types.SimpleNamespace(
        name="elbow_left",
        actuators=[],
        get_current=MagicMock(side_effect=BusTimeout("Did not receive response")),
    )
    healthy = types.SimpleNamespace(
        name="tilt_forward_motor",
        actuators=[],
        get_current=MagicMock(return_value=42),
    )
    module = _load_motor_current([failing, healthy])
    node = module.MotorCurrent()

    node.publish_motor_current()  # must not raise

    node.motor_current_publisher.publish.assert_called_once()
    message = node.motor_current_publisher.publish.call_args.args[0]
    assert message.name == "tilt_forward_motor"
    assert node._read_failures == 1
    assert node._read_failure_counts == {"elbow_left": 1}
    node.get_logger().warn.assert_called_once()


def test_bus_timeout_of_a_serial_actuator_skips_only_that_actuator():
    class BusTimeout(Exception):
        pass

    failing_actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        bricklet=None,
        pin=1,
        uid="AAA",
        get_current=MagicMock(side_effect=BusTimeout("no answer")),
        get_voltage=MagicMock(side_effect=BusTimeout("no answer")),
    )
    healthy_actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        bricklet=None,
        pin=2,
        uid="BBB",
        get_current=MagicMock(return_value=120),
        get_voltage=MagicMock(return_value=11.9),
    )
    motor = types.SimpleNamespace(
        name="shoulder_vertical_left",
        actuators=[failing_actuator, healthy_actuator],
        get_current=MagicMock(return_value=90),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()

    node.publish_motor_current()
    node.publish_motor_current()  # the slow tick carries the diagnostics batch

    node.motor_current_publisher.publish.assert_called()
    assert node._read_failure_counts.get("shoulder_vertical_left", 0) >= 1
    # the healthy actuator was still read on the slow tick
    assert healthy_actuator.get_current.call_count >= 1
    assert healthy_actuator.get_voltage.call_count >= 1


def test_single_actuator_reuses_current_and_voltage_without_duplicate_bus_reads():
    actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        bricklet=None,
        pin=1,
        uid="FEET1",
        get_current=MagicMock(return_value=140),
        get_voltage=MagicMock(return_value=11.8),
        get_temperature=MagicMock(return_value=36),
    )
    motor = types.SimpleNamespace(
        name="turn_head_motor",
        actuators=[actuator],
        get_current=MagicMock(side_effect=actuator.get_current),
        get_current_position=MagicMock(return_value=0),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()
    node.motor_current_publisher.get_subscription_count = MagicMock(return_value=1)

    with patch.object(module.motor_client, "report_diagnostics_telemetry") as report:
        node.publish_motor_current()  # fast tick
        node.publish_motor_current()  # slow tick

    # Slow tick must NOT call get_current/get_voltage a second time for the same actuator.
    assert actuator.get_current.call_count == 2  # once per tick, not 3
    assert actuator.get_voltage.call_count == 1  # once on slow tick, not 2
    report.assert_called_once()
    batch = report.call_args.args[0]
    assert batch == [{"uid": "FEET1", "pin": 1, "voltage": 11.8, "current": 140.0}]


def test_fast_tick_skipped_when_no_ros_subscribers_while_slow_telemetry_still_runs():
    actuator = types.SimpleNamespace(
        is_connected=lambda: True,
        bricklet=None,
        pin=3,
        uid="FEET2",
        get_current=MagicMock(return_value=110),
        get_voltage=MagicMock(return_value=11.7),
        get_temperature=MagicMock(return_value=35),
    )
    motor = types.SimpleNamespace(
        name="elbow_left",
        actuators=[actuator],
        get_current=MagicMock(side_effect=actuator.get_current),
        get_current_position=MagicMock(return_value=0),
    )
    module = _load_motor_current([motor])
    node = module.MotorCurrent()
    node.motor_current_publisher.get_subscription_count = MagicMock(return_value=0)

    with patch.object(module.motor_client, "report_diagnostics_telemetry") as report:
        node.publish_motor_current()  # fast tick -> skipped (0 subscribers)
        assert actuator.get_current.call_count == 0
        assert actuator.get_voltage.call_count == 0
        assert actuator.get_temperature.call_count == 0
        report.assert_not_called()

        node.publish_motor_current()  # slow tick -> still sends Flask diagnostics batch
        assert actuator.get_current.call_count == 1
        assert actuator.get_voltage.call_count == 1
        assert actuator.get_temperature.call_count == 0
        report.assert_called_once()
    node.motor_current_publisher.publish.assert_not_called()
