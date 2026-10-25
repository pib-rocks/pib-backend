"""ROS joint trajectory unit contract tests (PR-2048)."""

from __future__ import annotations

import importlib.util
import math
import sys
import types
from pathlib import Path
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


class FakeMotor:
    def __init__(
        self,
        name: str,
        *,
        rotation_range_min: int = -9000,
        rotation_range_max: int = 9000,
    ) -> None:
        self.name = name
        self.rotation_range_min = rotation_range_min
        self.rotation_range_max = rotation_range_max
        self._position = 0

    def set_position(self, position: int) -> bool:
        self._position = int(position)
        return True

    def get_position(self) -> int:
        return self._position


class FakeJointTrajectoryPoint:
    def __init__(self) -> None:
        self.positions: list[float] = []
        self.time_from_start = None
        self.velocities: list[float] = []
        self.accelerations: list[float] = []
        self.effort: list[float] = []


class FakeJointTrajectory:
    def __init__(self) -> None:
        self.joint_names: list[str] = []
        self.points: list[FakeJointTrajectoryPoint] = []


def _load_motor_control_helpers():
    elbow = FakeMotor("elbow_left")
    lookup = {"elbow_left": [elbow]}

    trajectory_msgs = _module("trajectory_msgs")
    trajectory_msgs_msg = _module(
        "trajectory_msgs.msg",
        JointTrajectory=FakeJointTrajectory,
        JointTrajectoryPoint=FakeJointTrajectoryPoint,
    )
    trajectory_msgs.msg = trajectory_msgs_msg

    datatypes_srv = _module(
        "datatypes.srv",
        ApplyJointTrajectory=MagicMock(),
        ApplyMotorSettings=MagicMock(),
        GetJointPosition=MagicMock(),
    )
    datatypes_msg = _module(
        "datatypes.msg", MotorSettings=MagicMock(), SolidStateRelayState=MagicMock()
    )

    pib_motors = _module("pib_motors")
    pib_motors_motor = _module(
        "pib_motors.motor", name_to_motors=lookup, motors=[elbow], Motor=FakeMotor
    )
    pib_motors.motor = pib_motors_motor
    pib_motors_startup = _module(
        "pib_motors.startup_pose_executor", StartupPoseExecutor=MagicMock()
    )
    pib_motors_trajectory = _module(
        "pib_motors.trajectory_executor",
        TrajectoryExecutor=MagicMock(),
        is_software_trajectory=lambda _jt: False,
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
        "pib_motors.startup_pose_executor": pib_motors_startup,
        "pib_motors.trajectory_executor": pib_motors_trajectory,
        "trajectory_msgs": trajectory_msgs,
        "trajectory_msgs.msg": trajectory_msgs_msg,
    }

    spec = importlib.util.spec_from_file_location(
        "motor_control_ros_units_under_test", MOTOR_CONTROL_PATH
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, dependencies):
        spec.loader.exec_module(module)
    return module, lookup, elbow


@pytest.fixture
def motor_control_module():
    return _load_motor_control_helpers()


def test_as_joint_trajectory_publishes_radians(motor_control_module) -> None:
    module, _, _ = motor_control_module
    message = module.as_joint_trajectory("elbow_left", 9000)
    assert message.points[0].positions[0] == pytest.approx(math.pi / 2.0)


def test_service_conversion_produces_centidegrees_for_executor(
    motor_control_module,
) -> None:
    module, lookup, _ = motor_control_module
    jt = FakeJointTrajectory()
    jt.joint_names = ["elbow_left"]
    point = FakeJointTrajectoryPoint()
    point.positions.append(math.pi / 2.0)
    jt.points.append(point)

    converted = module.joint_trajectory_radians_to_centidegrees(jt, lookup)
    assert converted.points[0].positions[0] == pytest.approx(9000.0)


def test_out_of_range_radian_request_is_rejected(motor_control_module) -> None:
    module, lookup, _ = motor_control_module
    jt = FakeJointTrajectory()
    jt.joint_names = ["elbow_left"]
    point = FakeJointTrajectoryPoint()
    point.positions.append(2.0)
    jt.points.append(point)

    with pytest.raises(ValueError, match="outside allowed range"):
        module.joint_trajectory_radians_to_centidegrees(jt, lookup)


def test_non_finite_radian_request_is_rejected(motor_control_module) -> None:
    module, lookup, _ = motor_control_module
    jt = FakeJointTrajectory()
    jt.joint_names = ["elbow_left"]
    point = FakeJointTrajectoryPoint()
    point.positions.append(float("inf"))
    jt.points.append(point)

    with pytest.raises(ValueError, match="finite"):
        module.joint_trajectory_radians_to_centidegrees(jt, lookup)


def test_apply_joint_trajectory_passes_centidegrees_to_motor(
    motor_control_module,
) -> None:
    module, _lookup, elbow = motor_control_module
    node = module.MotorControl.__new__(module.MotorControl)
    node.get_logger = MagicMock(return_value=MagicMock())
    node.joint_trajectory_publisher = MagicMock()
    node.trajectory_executor = MagicMock()

    request = MagicMock()
    jt = FakeJointTrajectory()
    jt.joint_names = ["elbow_left"]
    point = FakeJointTrajectoryPoint()
    point.positions.append(math.pi / 4.0)
    jt.points.append(point)
    request.joint_trajectory = jt

    response = MagicMock()
    response.successful = True
    result = module.MotorControl.apply_joint_trajectory(node, request, response)

    assert result.successful is True
    assert elbow.get_position() == 4500
    published = node.joint_trajectory_publisher.publish.call_args[0][0]
    assert published.points[0].positions[0] == pytest.approx(math.pi / 4.0)
