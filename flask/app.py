import os
import uuid
from flask import Flask, jsonify, request
from flask_sqlalchemy import SQLAlchemy
from flask_marshmallow import Marshmallow

app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname("/home/pib/pib_data/"))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'pibdata.db')
db = SQLAlchemy(app)
ma = Marshmallow(app)

class Personality(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), nullable=False)
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
    resolution = db.Column(db.String(20), nullable=False)
    refreshRate = db.Column(db.Float, nullable=False)
    qualityFactor = db.Column(db.Integer, nullable=False)
    isActive = db.Column(db.Boolean, nullable=False)

    def __init__(self, resolution, refreshRate, qualityFactor, isActive):
        self.resolution = resolution
        self.refreshRate = refreshRate
        self.qualityFactor = qualityFactor
        self.isActive = isActive


class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id',)

class CameraSettingsSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)

personality_schema = PersonalitySchema()
personalities_schema = PersonalitySchema(many=True)
camera_settings_schema = CameraSettingsSchema()

@app.route('/voice-assistant/personality')
def get_all_personalities():
    all_personalities = Personality.query.all()
    return jsonify({"voiceAssistantPersonalities": personalities_schema.dump(all_personalities)})

@app.route('/voice-assistant/personality/<string:uuid>', methods=['GET'])
def get_personality_by_id(uuid):
    getPersonality = Personality.query.filter(Personality.personalityId == uuid).first()
    return personality_schema.jsonify(getPersonality)

@app.route('/voice-assistant/personality', methods=['POST'])
def create_personality():
    personality = Personality(request.json.get('name'), request.json.get('gender'), request.json.get('pauseThreshold'))
    personality.personalityId = str(uuid.uuid4())
    db.session.add(personality)
    db.session.commit()
    returnPersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first()
    return jsonify(personality_schema.dump(returnPersonality)), 201

@app.route('/voice-assistant/personality/<string:uuid>', methods=['PUT'])
def update_personality(uuid):
    personality = Personality(request.json.get('name'), uuid, request.json.get('gender'), request.json.get('description'), request.json.get('pauseThreshold'))
    updatePersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first()
    updatePersonality.name = personality.name
    updatePersonality.description = personality.description
    updatePersonality.gender = personality.gender
    updatePersonality.pauseThreshold = personality.pauseThreshold
    db.session.add(updatePersonality)
    db.session.commit()
    updatePersonality = Personality.query.filter(Personality.personalityId == personality.personalityId).first()
    return personality_schema.jsonify(updatePersonality)

@app.route('/voice-assistant/personality/<string:uuid>', methods=['DELETE'])
def delete_personality(uuid):
    deletePersonality = Personality.query.filter(Personality.personalityId == uuid).first()
    db.session.delete(deletePersonality)
    db.session.commit()
    return '', 204

@app.route('/camera-settings', methods=['GET'])
def get_camera_settings():
    cameraSettings = CameraSettings.query.all()
    return camera_settings_schema.jsonify(cameraSettings[0])


@app.route('/camera-settings', methods=['PUT'])
def update_camera_settings():
    newCameraSettings = CameraSettings(request.json.get('resolution'), request.json.get('refreshRate'), request.json.get('qualityFactor'), request.json.get('isActive'))
    updateCameraSettings = CameraSettings.query.filter(CameraSettings.id == 1).first()
    updateCameraSettings.resolution = newCameraSettings.resolution
    updateCameraSettings.refreshRate = newCameraSettings.refreshRate
    updateCameraSettings.qualityFactor = newCameraSettings.qualityFactor
    updateCameraSettings.isActive = newCameraSettings.isActive
    db.session.add(updateCameraSettings)
    db.session.commit()
    response = CameraSettings.query.filter(CameraSettings.id == 1).first()
    print(response.isActive)
    return camera_settings_schema.dump(response)

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")