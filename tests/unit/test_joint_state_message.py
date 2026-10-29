"""PR-2132: the publisher must write the fields JointState really has."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
if str(PIB_MOTORS_DIR) not in sys.path:
    sys.path.insert(0, str(PIB_MOTORS_DIR))
MODULE_PATH = (
    REPO_ROOT
    / "ros_packages"
    / "motors"
    / "pib_motors"
    / "pib_motors"
    / "joint_state_sweep.py"
)


class _JointStateLike:
    """Mirror of sensor_msgs/JointState in the robot image.

    Fields read with ``JointState.get_fields_and_field_types()``:
    ``header, name, position, velocity, effort``. There is no ``joint_names`` -
    writing it raised AttributeError in the publish callback and killed the
    motor_control process the first time a sweep returned data.
    """

    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "pib_motors.joint_state_sweep", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolve the defining module, so register it before exec.
    sys.modules["pib_motors.joint_state_sweep"] = module
    spec.loader.exec_module(module)
    return module


def test_fill_joint_state_uses_name_and_position():
    """Load-bearing: 'joint_names' does not exist and crashes the publisher."""
    module = _load_module()
    message = _JointStateLike()

    written = module.fill_joint_state(
        message, {"turn_head_motor": 0, "tilt_forward_motor": 900}
    )

    assert written == 2
    assert message.name == ["tilt_forward_motor", "turn_head_motor"]
    assert len(message.position) == 2
    assert not hasattr(message, "joint_names")


def test_fill_joint_state_is_empty_for_an_empty_sweep():
    module = _load_module()
    message = _JointStateLike()

    assert module.fill_joint_state(message, {}) == 0
    assert message.name == []
    assert message.position == []
