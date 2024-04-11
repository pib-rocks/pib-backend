from model.motor_model import Motor
from model.bricklet_model import Bricklet
from model.bricklet_pin_model import BrickletPin
from typing import Any
from app.app import db


def get_all_motors() -> list[Motor]:
    return Motor.query.all()


def get_motor_by_name(motor_name) -> Motor:
    return Motor.query.filter(Motor.name == motor_name).one()


def set_motor_settings(motor_name: str, motor_settings_dto: dict[str, Any]):
    motor = get_motor_by_name(motor_name)
    motor.pulse_width_min = motor_settings_dto['pulse_width_min']
    motor.pulse_width_max = motor_settings_dto['pulse_width_max']
    motor.rotation_range_min = motor_settings_dto['rotation_range_min']
    motor.rotation_range_max = motor_settings_dto['rotation_range_max']
    motor.velocity = motor_settings_dto['velocity']
    motor.acceleration = motor_settings_dto['acceleration']
    motor.deceleration = motor_settings_dto['deceleration']
    motor.period = motor_settings_dto['period']
    motor.turned_on = motor_settings_dto['turned_on']
    motor.visible = motor_settings_dto['visible']
    motor.invert = motor_settings_dto['invert']
    db.session.flush()
    return motor


def set_bricklet_pins(motor_name, bricklet_pin_dtos):
    motor = get_motor_by_name(motor_name)
    motor.bricklet_pins.clear()
    for dto in bricklet_pin_dtos:
        bricklet_uid = dto['bricklet']['uid']
        bricklet = Bricklet.query.filter(Bricklet.uid == bricklet_uid).one()
        bricklet_pin = BrickletPin(pin=dto['pin'])
        bricklet_pin.bricklet = bricklet
        bricklet_pin.motor = motor
        bricklet_pin.invert = dto['invert']
        db.session.add(bricklet_pin)
    db.session.flush()
    return motor