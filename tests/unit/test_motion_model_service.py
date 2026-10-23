"""Motion model chains, limits, and prompt text from the motor and pose tables."""

import pytest

from app.app import db
from model.motor_model import Motor
from model.motor_position_model import MotorPosition
from model.pose_model import Pose
from model.util import generate_uuid
from service.motion_model_service import (
    build_motion_model,
    get_motion_model,
    render_prompt,
)


@pytest.fixture()
def motors_only(app_ctx):
    Motor.query.delete()
    MotorPosition.query.delete()
    Pose.query.delete()
    db.session.commit()
    yield


def _add_motor(
    name: str,
    *,
    rotation_range_min: int = -9000,
    rotation_range_max: int = 9000,
    upper_endpoint: str = "max",
    visible: bool = True,
) -> Motor:
    motor = Motor(
        name=name,
        pulse_width_min=700,
        pulse_width_max=2500,
        rotation_range_min=rotation_range_min,
        rotation_range_max=rotation_range_max,
        velocity=100,
        acceleration=100,
        deceleration=100,
        period=100,
        turned_on=True,
        visible=visible,
        upper_endpoint=upper_endpoint,
    )
    db.session.add(motor)
    db.session.flush()
    return motor


def test_chains_follow_the_declared_motor_order(motors_only):
    _add_motor("wrist_left")
    _add_motor("turn_head_motor")
    _add_motor("elbow_left")
    _add_motor("shoulder_vertical_left")
    db.session.commit()

    model = build_motion_model()
    head_motors = [joint["motor"] for joint in model["chains"]["head"]["joints"]]
    left_arm_motors = [
        joint["motor"] for joint in model["chains"]["left_arm"]["joints"]
    ]

    assert head_motors == ["turn_head_motor"]
    assert left_arm_motors == ["shoulder_vertical_left", "elbow_left", "wrist_left"]


def test_upper_end_value_follows_upper_endpoint(motors_only):
    _add_motor(
        "turn_head_motor",
        rotation_range_min=-9000,
        rotation_range_max=9000,
        upper_endpoint="max",
    )
    _add_motor(
        "tilt_forward_motor",
        rotation_range_min=-2000,
        rotation_range_max=4500,
        upper_endpoint="min",
    )
    db.session.commit()

    model = build_motion_model()
    by_name = {joint["motor"]: joint for joint in model["chains"]["head"]["joints"]}
    assert by_name["turn_head_motor"]["upperEndValue"] == 9000
    assert by_name["turn_head_motor"]["lowerEndValue"] == -9000
    assert by_name["tilt_forward_motor"]["upperEndValue"] == -2000
    assert by_name["tilt_forward_motor"]["lowerEndValue"] == 4500

    prompt = render_prompt(model)
    assert "tilt_forward_motor -2000..4500 (upper end -2000)" in prompt


def test_hidden_motor_is_marked_in_the_prompt(motors_only):
    _add_motor("turn_head_motor", visible=False)
    db.session.commit()

    prompt = render_prompt(build_motion_model())
    assert "turn_head_motor" in prompt
    assert "(hidden from the UI)" in prompt


def test_unknown_motor_lands_in_other_chain(motors_only):
    _add_motor("custom_servo")
    db.session.commit()

    model = build_motion_model()
    assert model["chains"]["other"]["joints"][0]["motor"] == "custom_servo"


def test_empty_database_yields_empty_chains_and_units_only_prompt(motors_only):
    Motor.query.delete()
    db.session.commit()

    model = build_motion_model()
    assert model["chains"] == {}
    assert model["postures"] == {}
    prompt = render_prompt(model)
    assert prompt.startswith("Body (centidegrees:")
    assert "Head:" not in prompt


def test_postures_are_read_from_the_pose_table(motors_only):
    _add_motor("turn_head_motor")
    pose = Pose(pose_id=generate_uuid(), name="Startup/Resting", deletable=False)
    db.session.add(pose)
    db.session.flush()
    db.session.add(
        MotorPosition(pose_id=pose.id, motor_name="turn_head_motor", position=0)
    )
    db.session.commit()

    model = build_motion_model()
    assert model["postures"] == {"Startup/Resting": {"turn_head_motor": 0}}
    prompt = render_prompt(model)
    assert "Postures you can apply: Startup/Resting." in prompt
    assert "Startup/Resting: turn_head_motor=0" in prompt

    payload = get_motion_model()
    assert payload["prompt"] == prompt
    assert payload["schemaVersion"] == 1
