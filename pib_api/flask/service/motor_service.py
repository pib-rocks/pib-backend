from model.motor_model import Motor
from model.controller_model import Controller
from typing import Any, List
from app.app import db


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
        raise ValueError("Controller requires a number or address")
    motor.controller = controller
    motor.channel = channel
    db.session.flush()
    return motor
