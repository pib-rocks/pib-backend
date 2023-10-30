import os
import uuid
from flask import Flask, jsonify, request, abort
from flask_sqlalchemy import SQLAlchemy
from flask_marshmallow import Marshmallow
from flask_cors import CORS

app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname("/home/pib/pib_data/"))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'pibdata.db')
db = SQLAlchemy(app)
ma = Marshmallow(app)
CORS(app)

class Personality(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), nullable=False, unique=True)
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pauseThreshold = db.Column(db.Float, nullable=False)
    def __init__(self, *args):
        if len(args) == 3:
            self.eq3(args)
        if len(args) == 5:
            self.eq5(args)

    def eq3(self, args):
        self.name = args[0]
        self.personalityIdd = str(uuid.uuid4())
        self.description = ""
        self.gender = args[1]
        self.pauseThreshold = args[2]

    def eq5(self, args):
        self.name = args[0]
        self.personalityId = args[1]
        self.gender = args[2]
        self.description = args[3]
        self.pauseThreshold = args[4]

class CameraSettings(db.Model):
    __tablename__ = "cameraSettings"
    id = db.Column(db.Integer, primary_key=True)
    resolution = db.Column(db.String(3), nullable=False)
    refreshRate = db.Column(db.Float, nullable=False)
    qualityFactor = db.Column(db.Integer, nullable=False)
    isActive = db.Column(db.Boolean, nullable=False)
    resX = db.Column(db.Integer, nullable=False)
    resY = db.Column(db.Integer, nullable=False)

    def __init__(self, resolution, refreshRate, qualityFactor, isActive, resX, resY):
        self.resolution = resolution
        self.refreshRate = refreshRate
        self.qualityFactor = qualityFactor
        self.isActive = isActive
        self.resX = resX
        self.resY = resY

class Motor(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name= db.Column(db.String(255), nullable=False, unique=True)
    pulseWidthMin = db.Column(db.Integer, nullable=False)
    pulseWidthMax = db.Column(db.Integer, nullable=False)
    rotationRangeMin = db.Column(db.Integer, nullable=False)
    rotationRangeMax = db.Column(db.Integer, nullable=False)
    velocity = db.Column(db.Integer, nullable=False)
    acceleration = db.Column(db.Integer, nullable=False)
    deceleration = db.Column(db.Integer, nullable=False)
    period = db.Column(db.Integer, nullable=False)
    turnedOn = db.Column(db.Boolean, nullable=False)
    effort = db.Column(db.Integer, nullable=True)
    def __init__(self, *args):
        self.name = args[0]
        self.pulseWidthMin = args[1]
        self.pulseWidthMax = args[2]
        self.rotationRangeMin = args[3]
        self.rotationRangeMax = args[4]
        self.velocity = args[5]
        self.acceleration = args[6]
        self.deceleration = args[7]
        self.period = args[8]
        self.turnedOn = args[9]
        if len(args) > 10:
            self.effort = args[10]


class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id', 'personalityId',)

class CameraSettingsSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)

class MotorSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Motor
        exclude = ('id', 'effort')

personality_schema = PersonalitySchema()
personalities_schema = PersonalitySchema(many=True)
camera_settings_schema = CameraSettingsSchema()
motor_schema = MotorSchema()
motors_schema = MotorSchema(many=True)

@app.route('/voice-assistant/personality')
def get_all_personalities():
    all_personalities = Personality.query.all()
    try:
        return jsonify({"voiceAssistantPersonalities": personalities_schema.dump(all_personalities)})
    except:
        abort(500)

@app.route('/voice-assistant/personality/<string:uuid>', methods=['GET'])
def get_personality_by_id(uuid):
    getPersonality = Personality.query.filter(Personality.personalityId == uuid).first_or_404()

    try:
        return personality_schema.dump(getPersonality)
    except:
        abort(500)

@app.route('/voice-assistant/personality', methods=['POST'])
def create_personality():
    error = personality_schema.validate(request.json)
    if error:
        return error, 400
    
    personality = Personality(request.json.get('name'), request.json.get('gender'), request.json.get('pauseThreshold'))
    personality.personalityId = str(uuid.uuid4())

    db.session.add(personality)
    db.session.commit()

    returnPersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first_or_404()

    try:
        return jsonify(personality_schema.dump(returnPersonality)), 201
    except:
        abort(500)

@app.route('/voice-assistant/personality/<string:uuid>', methods=['PUT'])
def update_personality(uuid):
    error = personality_schema.validate(request.json)
    if error:
        return error, 400
    
    personality = Personality(request.json.get('name'), uuid, request.json.get('gender'), request.json.get('description'), request.json.get('pauseThreshold'))

    updatePersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first_or_404()
    updatePersonality.name = personality.name
    updatePersonality.description = personality.description
    updatePersonality.gender = personality.gender
    updatePersonality.pauseThreshold = personality.pauseThreshold

    db.session.add(updatePersonality)
    db.session.commit()

    updatePersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first_or_404()

    try:
        return personality_schema.dump(updatePersonality)
    except:
        abort(500)


@app.route('/voice-assistant/personality/<string:uuid>', methods=['DELETE'])
def delete_personality(uuid):
    deletePersonality = Personality.query.filter(Personality.personalityId == uuid).first_or_404()

    db.session.delete(deletePersonality)
    db.session.commit()

    return '', 204

@app.route('/camera-settings', methods=['GET'])
def get_camera_settings():
    cameraSettings = CameraSettings.query.all()
    
    try:
        return camera_settings_schema.dump(cameraSettings[0])
    except:
        abort(500)


@app.route('/camera-settings', methods=['PUT'])
def update_camera_settings():

    error = camera_settings_schema.validate(request.json)
    if error:
        return error, 400

    newCameraSettings = CameraSettings(request.json.get('resolution'), request.json.get('refreshRate'), request.json.get('qualityFactor'), request.json.get('isActive'), request.json.get('resX'), request.json.get('resY'))
    updateCameraSettings = CameraSettings.query.filter(CameraSettings.id == 1).first_or_404()
    updateCameraSettings.resolution = newCameraSettings.resolution
    updateCameraSettings.refreshRate = newCameraSettings.refreshRate
    updateCameraSettings.qualityFactor = newCameraSettings.qualityFactor
    updateCameraSettings.isActive = newCameraSettings.isActive
    updateCameraSettings.resX = newCameraSettings.resX
    updateCameraSettings.resY = newCameraSettings.resY

    db.session.add(updateCameraSettings)
    db.session.commit()

    response = CameraSettings.query.filter(CameraSettings.id == 1).first_or_404()
    try:
        return camera_settings_schema.dump(response)
    except:
        abort(500)

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error':"Entity not found. Please check the ID."}), 404

@app.errorhandler(500)
def not_found(error):
    return jsonify({'error': "Internal Server Error, please try later again."}), 500

@app.route('/motor-settings/<string:name>', methods=['GET'])
def get_motor(name):
    motor = Motor.query.filter(Motor.name == name).first()
    return motor_schema.dump(motor)

@app.route('/motor-settings/', methods=['GET'])
def get_motors():
    motors = Motor.query.all()
    return jsonify({"motorSettings": motors_schema.dump(motors)})

@app.route('/motor-settings/', methods=['PUT'])
def update_motor():
    updateMotor = Motor(request.json.get('motor_name'), request.json.get('pulse_width_min'), request.json.get('pulse_width_max'), request.json.get('rotation_range_min'), request.json.get('rotation_range_max'), request.json.get('velocity'), request.json.get('acceleration'), request.json.get('deceleration'), request.json.get('period'), request.json.get('turned_on'))
    motor = Motor.query.filter(Motor.name == updateMotor.name).first()
    motor.pulseWidthMin = updateMotor.pulseWidthMin
    motor.pulseWidthMax = updateMotor.pulseWidthMax
    motor.rotationRangeMin = updateMotor.rotationRangeMin
    motor.rotationRangeMax = updateMotor.rotationRangeMax
    motor.velocity = updateMotor.velocity
    motor.acceleration = updateMotor.acceleration
    motor.deceleration = updateMotor.deceleration
    motor.period = updateMotor.period
    motor.turnedOn = updateMotor.turnedOn
    #motor.effort = updateMotor.effort
    db.session.add(motor)
    db.session.commit()
    return motor_schema.dump(motor)

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")