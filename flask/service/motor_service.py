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
    motor.pulseWidthMin = motor_settings_dto['pulseWidthMin']
    motor.pulseWidthMax = motor_settings_dto['pulseWidthMax']
    motor.rotationRangeMin = motor_settings_dto['rotationRangeMin']
    motor.rotationRangeMax = motor_settings_dto['rotationRangeMax']
    motor.velocity = motor_settings_dto['velocity']
    motor.acceleration = motor_settings_dto['acceleration']
    motor.deceleration = motor_settings_dto['deceleration']
    motor.period = motor_settings_dto['period']
    motor.turnedOn = motor_settings_dto['turnedOn']
    motor.visible = motor_settings_dto['visible']
    db.session.flush()
    return motor


def set_bricklet_pins(motor_name, bricklet_pin_dtos):
    motor = get_motor_by_name(motor_name)
    motor.brickletPins.clear()
    for dto in bricklet_pin_dtos:
        bricklet_uid = dto['bricklet']['uid']
        bricklet = Bricklet.query.filter(Bricklet.uid == bricklet_uid).one()
        bricklet_pin = BrickletPin(pin=dto['pin'])
        bricklet_pin.bricklet = bricklet
        bricklet_pin.motor = motor
        db.session.add(bricklet_pin)
    db.session.flush()
    return motor