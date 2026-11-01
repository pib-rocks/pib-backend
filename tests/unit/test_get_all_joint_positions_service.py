"""PR-2113: synchronous batch joint positions service on motor_control."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOTORS_PKG = REPO_ROOT / "ros_packages" / "motors"
PIB_MOTORS_ROOT = MOTORS_PKG / "pib_motors"
MOTOR_CONTROL_PATH = MOTORS_PKG / "motors" / "motor_control.py"

if str(PIB_MOTORS_ROOT) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_ROOT))


def _module(name: str, **attributes: object) -> types.ModuleType:
    module = types.ModuleType(name)
    for attribute, value in attributes.items():
        setattr(module, attribute, value)
    return module


class FakeMotorResult:
    def __init__(self) -> None:
        self.motor_name = ""
        self.successful = False
        self.error_code = ""
        self.detail = ""


class FakeHeader:
    def __init__(self) -> None:
        self.stamp = SimpleNamespace(sec=0, nanosec=0)


def _motor_control_ros_import_stubs() -> dict[str, types.ModuleType]:
    import pib_motors.joint_state_sweep as real_joint_sweep
    import pib_motors.perf_trace as real_perf_trace

    sensor_msgs = _module("sensor_msgs")
    sensor_msgs_msg = _module("sensor_msgs.msg", JointState=MagicMock())
    sensor_msgs.msg = sensor_msgs_msg
    std_msgs_msg = _module("std_msgs.msg", Header=FakeHeader)
    return {
        "pib_motors.perf_trace": real_perf_trace,
        "pib_motors.joint_state_sweep": real_joint_sweep,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs.msg": std_msgs_msg,
    }


def _load_motor_control_for_batch(lookup: dict, motors_list: list):
    trajectory_msgs = _module("trajectory_msgs")
    trajectory_msgs_msg = _module(
        "trajectory_msgs.msg",
        JointTrajectory=MagicMock(),
        JointTrajectoryPoint=MagicMock(),
    )
    trajectory_msgs.msg = trajectory_msgs_msg

    datatypes_srv = _module(
        "datatypes.srv",
        ApplyJointTrajectory=MagicMock(),
        ApplyMotorSettings=MagicMock(),
        GetJointPosition=MagicMock(),
        GetAllJointPositions=MagicMock(),
    )
    datatypes_msg = _module(
        "datatypes.msg",
        MotorSettings=MagicMock(),
        SolidStateRelayState=MagicMock(),
        MotorResult=FakeMotorResult,
    )

    pib_motors = _module("pib_motors")
    pib_motors_motor = _module(
        "pib_motors.motor", name_to_motors=lookup, motors=motors_list, Motor=MagicMock
    )
    pib_motors.motor = pib_motors_motor

    import pib_motors.joint_trajectory_command as joint_trajectory_command
    import pib_motors.trajectory_executor as trajectory_executor

    pib_motors_trajectory = _module(
        "pib_motors.trajectory_executor",
        TrajectoryExecutor=trajectory_executor.TrajectoryExecutor,
        is_software_trajectory=trajectory_executor.is_software_trajectory,
        validate_software_trajectory_schedule=(
            trajectory_executor.validate_software_trajectory_schedule
        ),
    )
    pib_motors_joint_cmd = _module(
        "pib_motors.joint_trajectory_command",
        **{
            name: getattr(joint_trajectory_command, name)
            for name in (
                "MotorOutcome",
                "is_legacy_zip_shape",
                "iter_single_point_centideg_targets",
                "legacy_zip_outcomes",
                "outcomes_to_success",
                "validate_radians_trajectory",
            )
        },
    )

    rclpy = _module("rclpy", init=MagicMock(), spin=MagicMock(), shutdown=MagicMock())

    class FakeNode:
        def __init__(self, name: str = "") -> None:
            self.name = name

    rclpy_node = _module("rclpy.node", Node=FakeNode)
    rclpy.node = rclpy_node

    import pib_motors.units as real_units

    dependencies = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "pib_motors.units": real_units,
        "datatypes.msg": datatypes_msg,
        "datatypes.srv": datatypes_srv,
        "pib_api_client": _module("pib_api_client", motor_client=MagicMock()),
        "pib_motors": pib_motors,
        "pib_motors.bricklet": _module(
            "pib_motors.bricklet",
            ipcon=MagicMock(),
            connected_enumerate=MagicMock(),
            solid_state_relay_bricklet=None,
        ),
        "pib_motors.motor": pib_motors_motor,
        "pib_motors.startup_pose_executor": _module(
            "pib_motors.startup_pose_executor", StartupPoseExecutor=MagicMock()
        ),
        "pib_motors.trajectory_executor": pib_motors_trajectory,
        "pib_motors.joint_trajectory_command": pib_motors_joint_cmd,
        "trajectory_msgs": trajectory_msgs,
        "trajectory_msgs.msg": trajectory_msgs_msg,
        **_motor_control_ros_import_stubs(),
    }

    spec = importlib.util.spec_from_file_location(
        "motor_control_batch_under_test", MOTOR_CONTROL_PATH
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, dependencies):
        spec.loader.exec_module(module)
    return module


def test_no_bus_returns_clear_error_without_hanging(monkeypatch):
    lookup = {"elbow_left": [SimpleNamespace(name="elbow_left")]}
    module = _load_motor_control_for_batch(lookup, [])

    from pib_motors import joint_state_sweep

    empty = joint_state_sweep.JointStateSweepResult({}, 0.0, 0, 0)
    monkeypatch.setattr(module, "sweep_feetech_joint_positions", lambda _motors: empty)

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=1, nanosec=2)
            )
        ),
        _motor_power_is_off=lambda: False,
    )
    handler = module.MotorControl.get_all_joint_positions.__get__(
        node, module.MotorControl
    )
    request = SimpleNamespace(joint_names=[])
    response = SimpleNamespace(
        header=None,
        successful=True,
        message="",
        results=[],
        positions=[],
    )

    handler(request, response)
    assert response.successful is False
    assert "no sweepable actuator family" in response.message
    assert len(response.results) == 1
    assert response.results[0].error_code == "bus_unavailable"


def test_one_silent_joint_keeps_other_measurements(monkeypatch):
    lookup = {
        "good_joint": [SimpleNamespace(name="good_joint")],
        "silent_joint": [SimpleNamespace(name="silent_joint")],
    }
    module = _load_motor_control_for_batch(lookup, [])

    from pib_motors import joint_state_sweep

    sweep = joint_state_sweep.JointStateSweepResult({"good_joint": 1234}, 1.0, 1, 1)
    monkeypatch.setattr(module, "sweep_feetech_joint_positions", lambda _motors: sweep)

    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=3, nanosec=4)
            )
        ),
        _motor_power_is_off=lambda: False,
    )
    handler = module.MotorControl.get_all_joint_positions.__get__(
        node, module.MotorControl
    )
    request = SimpleNamespace(joint_names=["good_joint", "silent_joint"])
    response = SimpleNamespace(
        header=None,
        successful=False,
        message="",
        results=[],
        positions=[],
    )

    handler(request, response)
    assert response.successful is True
    assert response.positions == [1234, 0]
    assert response.results[0].successful is True
    assert response.results[1].successful is False
    assert response.results[1].error_code == "position_unavailable"


def _batch_call(monkeypatch, lookup, sweep_result, *, power_off: bool):
    module = _load_motor_control_for_batch(lookup, [])
    calls = []

    def fake_sweep(_motors):
        calls.append(1)
        return sweep_result

    monkeypatch.setattr(module, "sweep_feetech_joint_positions", fake_sweep)
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=1, nanosec=2)
            )
        ),
        _motor_power_is_off=lambda: power_off,
    )
    handler = module.MotorControl.get_all_joint_positions.__get__(
        node, module.MotorControl
    )
    response = SimpleNamespace(
        header=None, successful=True, message="", results=[], positions=[]
    )
    handler(SimpleNamespace(joint_names=[]), response)
    return response, calls


def test_power_off_answers_without_touching_the_bus(monkeypatch):
    """Load-bearing: the SSR-off case - 26 probes x 50 ms timeout per sweep."""
    from pib_motors import joint_state_sweep

    lookup = {"turn_head_motor": [SimpleNamespace(name="turn_head_motor")]}
    empty = joint_state_sweep.JointStateSweepResult({}, 0.0, 0, 0)

    response, calls = _batch_call(monkeypatch, lookup, empty, power_off=True)

    assert calls == [], "kein Sweep bei ausgeschalteter Versorgung"
    assert response.successful is False
    assert "motor power is off" in response.message
    assert response.results[0].error_code == "bus_unavailable"
    assert response.results[0].detail == response.message


def test_unreachable_ports_are_named_instead_of_claiming_no_actuators(monkeypatch):
    """A dead bus must not read like 'this robot has no sweepable actuators'."""
    from pib_motors import joint_state_sweep

    lookup = {"turn_head_motor": [SimpleNamespace(name="turn_head_motor")]}
    dead = joint_state_sweep.JointStateSweepResult({}, 0.0, 0, 0, {}, {}, 4)

    response, calls = _batch_call(monkeypatch, lookup, dead, power_off=False)

    assert calls == [1]
    assert response.successful is False
    assert "no Feetech bus reachable" in response.message
    assert "4 port(s)" in response.message
    assert response.results[0].error_code == "bus_unavailable"


def test_power_off_stops_the_joint_state_publisher(monkeypatch):
    """Load-bearing: no measurement attempts while the relay is off."""
    module = _load_motor_control_for_batch({"turn_head_motor": []}, [])
    calls = []
    published = []
    monkeypatch.setattr(
        module, "sweep_feetech_joint_positions", lambda _m: calls.append(1)
    )
    node = SimpleNamespace(
        _motor_power_is_off=lambda: True,
        joint_state_publisher=SimpleNamespace(
            publish=lambda msg: published.append(msg)
        ),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: SimpleNamespace(sec=1, nanosec=2)
            )
        ),
    )
    module.MotorControl._publish_joint_states.__get__(node, module.MotorControl)()

    assert calls == []
    assert published == []
