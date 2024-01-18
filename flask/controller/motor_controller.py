from model.motor_model import Motor
from model.bricklet_model import Bricklet
from model.bricklet_pin_model import BrickletPin
from schema.motor_schema import motor_schema, motors_schema, motor_settings_schema, motor_bricklet_pins_schema
from app.app import db
from flask import abort, jsonify, request

def get_all_motors():
    motors = Motor.query.all()
    try: return motors_schema.dump(motors)
    except: abort(500)

def get_motor(name: str):
    motor = Motor.query.filter(Motor.name == name).first_or_404()
    try: return motor_schema.dump(motor)
    except: abort(500)

def update_motor(name: str):
    abort(501)

def get_motor_settings(name: str):
    motor = Motor.query.filter(Motor.name == name).first_or_404()
    try: return motor_settings_schema.dump(motor)
    except: abort(500)

def update_motor_settings(name: str):
    abort(501)

def get_motor_bricklet_pins(name: str):
    motor = Motor.query.filter(Motor.name == name).first_or_404()
    try: return motor_bricklet_pins_schema.dump(motor)
    except: abort(500)

def update_motor_bricklet_pins(name: str):
    abort(501)