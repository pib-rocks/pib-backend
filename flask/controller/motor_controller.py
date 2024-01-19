from schema.motor_schema import motor_schema, motors_schema, motor_settings_schema, motor_bricklet_pins_schema
from service import motor_service
from app.app import db
from flask import request, jsonify


def get_all_motors():
    motors = motor_service.get_all_motors()
    return jsonify({'motors': motors_schema.dump(motors)})


def get_motor(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_schema.dump(motor)


def update_motor(name: str):
    bricklet_pin_dtos = motor_bricklet_pins_schema.load(request.json)['brickletPins']
    motor_settings_dto = motor_settings_schema.load(request.json)
    motor_service.set_bricklet_pins(name, bricklet_pin_dtos)
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    db.session.commit()
    return motor_schema.dump(motor)


def get_motor_settings(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_settings_schema.dump(motor)


def update_motor_settings(name: str):
    motor_settings_dto = motor_settings_schema.load(request.json)
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    db.session.commit()
    return motor_settings_schema.dump(motor)


def get_motor_bricklet_pins(name: str):
    motor = motor_service.get_motor_by_name(name)
    return motor_bricklet_pins_schema.dump(motor)


def update_motor_bricklet_pins(name: str):
    bricklet_pin_dtos = motor_bricklet_pins_schema.load(request.json)['brickletPins']
    motor = motor_service.set_bricklet_pins(name, bricklet_pin_dtos)
    db.session.commit()
    return motor_bricklet_pins_schema.dump(motor)