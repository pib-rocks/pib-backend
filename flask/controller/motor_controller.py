from model.motor_model import Motor
from schema.motor_schema import motor_schema, motors_schema
from app.app import db
from flask import abort, jsonify, request

def get_motor(name):
    motor = Motor.query.filter(Motor.name == name).first_or_404()
    try:
        return motor_schema.dump(motor)
    except:
        abort(500)

def get_motors():
    motors = Motor.query.all()
    try:
        return jsonify({"motorSettings": motors_schema.dump(motors)})
    except:
        abort(500)

def update_motor():
    error = motor_schema.validate(request.json)
    if error:
        return error, 400
    updateMotor = Motor(
        request.json.get('name'), 
        request.json.get('pulseWidthMin'), 
        request.json.get('pulseWidthMax'), 
        request.json.get('rotationRangeMin'), 
        request.json.get('rotationRangeMax'), 
        request.json.get('velocity'), 
        request.json.get('acceleration'), 
        request.json.get('deceleration'), 
        request.json.get('period'), 
        request.json.get('active'),
        request.json.get('turnedOn')
    )
    motor = Motor.query.filter(Motor.name == updateMotor.name).first_or_404()
    motor.pulseWidthMin = updateMotor.pulseWidthMin
    motor.pulseWidthMax = updateMotor.pulseWidthMax
    motor.rotationRangeMin = updateMotor.rotationRangeMin
    motor.rotationRangeMax = updateMotor.rotationRangeMax
    motor.velocity = updateMotor.velocity
    motor.acceleration = updateMotor.acceleration
    motor.deceleration = updateMotor.deceleration
    motor.period = updateMotor.period
    motor.turnedOn = updateMotor.turnedOn
    motor.active = updateMotor.active
    db.session.add(motor)
    db.session.commit()
    try:
        return motor_schema.dump(motor)
    except:
        abort(500)