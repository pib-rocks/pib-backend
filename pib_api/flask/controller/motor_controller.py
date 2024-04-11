from schema.motor_schema import motor_schema, motors_schema, motor_settings_schema, motor_bricklet_pins_schema
from service import motor_service
from app.app import db
from flask import request, jsonify, abort, Blueprint

bp = Blueprint('motor_controller', __name__)


@bp.route('/', methods=['GET'])
def get_all_motors():
    motors = motor_service.get_all_motors()
    try: return jsonify({'motors': motors_schema.dump(motors)})
    except Exception: abort(500)


@bp.route('/<string:name>', methods=['GET'])
def get_motor(name: str):
    motor = motor_service.get_motor_by_name(name)
    try: return motor_schema.dump(motor)
    except Exception: abort(500)


@bp.route('/<string:name>', methods=['PUT'])
def update_motor(name: str):
    bricklet_pin_dtos = motor_schema.load(request.json)['brickletPins']
    motor_settings_dto = motor_schema.load(request.json)
    motor_service.set_bricklet_pins(name, bricklet_pin_dtos)
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    db.session.commit()
    try: return motor_schema.dump(motor)
    except Exception: abort(500)


@bp.route('/<string:name>/settings', methods=['GET'])
def get_motor_settings(name: str):
    motor = motor_service.get_motor_by_name(name)
    try: return motor_settings_schema.dump(motor)
    except Exception: abort(500)


@bp.route('/<string:name>/settings', methods=['PUT'])
def update_motor_settings(name: str):
    motor_settings_dto = motor_settings_schema.load(request.json)
    motor = motor_service.set_motor_settings(name, motor_settings_dto)
    db.session.commit()
    try: return motor_settings_schema.dump(motor)
    except Exception: abort(500)


@bp.route('/<string:name>/bricklet-pins', methods=['GET'])
def get_motor_bricklet_pins(name: str):
    motor = motor_service.get_motor_by_name(name)
    try: return motor_bricklet_pins_schema.dump(motor)
    except Exception: abort(500)


@bp.route('/<string:name>/bricklet-pins', methods=['PUT'])
def update_motor_bricklet_pins(name: str):
    bricklet_pin_dtos = motor_bricklet_pins_schema.load(request.json)['brickletPins']
    motor = motor_service.set_bricklet_pins(name, bricklet_pin_dtos)
    db.session.commit()
    try: return motor_bricklet_pins_schema.dump(motor)
    except Exception: abort(500)