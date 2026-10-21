from typing import Any, List

from app.app import db
from default_pose_constants import CALIBRATION_POSE_NAME, STARTUP_POSE_NAME
from model.controller_model import Controller
from model.motor_model import Motor
from model.motor_position_model import MotorPosition
from model.pose_model import Pose


class MotorValidationError(Exception):
    """The request does not describe a valid change of a motor (PR-1978).

    A controller update that names neither a controller number nor an address is a
    bad request (400), not a server error.
    """


def get_all_motors() -> List[Motor]:
    return Motor.query.all()


def get_motor_by_name(motor_name) -> Motor:
    return Motor.query.filter(Motor.name == motor_name).one()


def set_motor_settings(motor_name: str, motor_settings_dto: Any):
    motor = get_motor_by_name(motor_name)
    motor.pulse_width_min = motor_settings_dto["pulse_width_min"]
    motor.pulse_width_max = motor_settings_dto["pulse_width_max"]
    motor.rotation_range_min = motor_settings_dto["rotation_range_min"]
    motor.rotation_range_max = motor_settings_dto["rotation_range_max"]
    motor.velocity = motor_settings_dto["velocity"]
    motor.acceleration = motor_settings_dto["acceleration"]
    motor.deceleration = motor_settings_dto["deceleration"]
    motor.period = motor_settings_dto["period"]
    motor.turned_on = motor_settings_dto["turned_on"]
    motor.visible = motor_settings_dto["visible"]
    motor.invert = motor_settings_dto["invert"]
    if "current_limit" in motor_settings_dto:
        motor.current_limit = motor_settings_dto["current_limit"]
    if "torque_limit" in motor_settings_dto:
        motor.torque_limit = motor_settings_dto["torque_limit"]
    db.session.flush()
    return motor


def set_motor_controller(motor_name: str, controller_dto: Any, channel: int):
    motor = get_motor_by_name(motor_name)
    number = controller_dto.get("number")
    address = controller_dto.get("address")
    query = Controller.query
    if number is not None:
        controller = query.filter(Controller.number == number).one()
    elif address is not None:
        controller = query.filter(Controller.address == address).one()
    else:
        raise MotorValidationError("Controller requires a number or address")
    motor.controller = controller
    motor.channel = channel
    db.session.flush()
    return motor


def calibrate_motor_zero(motor_name: str) -> dict[str, Any]:
    motor = get_motor_by_name(motor_name)
    for pose_name in (STARTUP_POSE_NAME, CALIBRATION_POSE_NAME):
        pose = Pose.query.filter_by(name=pose_name).one_or_none()
        if pose is None:
            continue
        motor_position = MotorPosition.query.filter_by(
            pose_id=pose.id, motor_name=motor_name
        ).one_or_none()
        if motor_position is not None:
            motor_position.position = 0
    db.session.flush()
    return {"name": motor.name, "calibrated": True}
