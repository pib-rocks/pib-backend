import os
import uuid
from flask import Flask, jsonify, request, abort
from flask_sqlalchemy import SQLAlchemy
from flask_marshmallow import Marshmallow, Schema, fields
from marshmallow import fields as ma_fields, Schema as ma_schema, ValidationError
from flask_cors import CORS


app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname("/home/pib/pib_data/"))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:////home/pib/pib_data/pibdata.db'
db = SQLAlchemy(app)
ma = Marshmallow(app)
CORS(app)

PROGRAM_DIR = "/home/pib/pib_data/program/code/python"

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
        self.personalityId = str(uuid.uuid4())
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
    resX = db.Column(db.Integer, nullable=False)
    resY = db.Column(db.Integer, nullable=False)

    def __init__(self, resolution, refreshRate, qualityFactor, resX, resY):
        self.resolution = resolution
        self.refreshRate = refreshRate
        self.qualityFactor = qualityFactor
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

def create_python_program(programNumber):
    open(PROGRAM_DIR + "/" + programNumber + ".py", "w").close()

def update_python_program(programNumber, code):
    file = open(PROGRAM_DIR + "/" + programNumber + ".py", "w")
    file.write(code)
    file.close()

def delete_python_program(programNumber):
    os.remove(PROGRAM_DIR + "/" + programNumber + ".py")

class Program(db.Model):
    __tablename__ = "program"
    id = db.Column(db.Integer, primary_key=True)
    name= db.Column(db.String(255), nullable=False, unique=True)
    codeVisual = db.Column(db.String(100000), nullable=False)
    programNumber = db.Column(db.String(50), nullable=False)

    folder_path = "/home/pib/programs/"

    def __init__(self, name, codeVisual = "{}"):
        self.name = name
        self.codeVisual = codeVisual
        self.programNumber = str(uuid.uuid4())

class Chat(db.Model):
    __tablename__ = "chat"
    id = db.Column(db.Integer, primary_key=True)
    chatId= db.Column(db.String(255), nullable=False, unique=True)
    topic = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), nullable=False, unique=True)
    def __init__(self, topic, personalityId):
        self.chatId = str(uuid.uuid4())
        self.topic = topic
        self.personalityId = personalityId

class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id',)

class ChatSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Chat
        exclude = ('id',)
        
class UploadChatSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Chat
        exclude = ('id', 'chatId',)


class UploadPersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id', 'personalityId')


class CameraSettingsSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)

class MotorSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Motor
        exclude = ('id', 'effort')

class ProgramSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Program

class ProgramCodeSchema(ma_schema):
    visual = ma_fields.String(required=True)
    python = ma_fields.String(required=True)

personality_schema = PersonalitySchema()
upload_personality_schema = UploadPersonalitySchema()
personalities_schema = PersonalitySchema(many=True)

camera_settings_schema = CameraSettingsSchema()

motor_schema = MotorSchema()
motors_schema = MotorSchema(many=True)

chat_schema = ChatSchema()
chats_schema = ChatSchema(many=True)
upload_chat_schema = UploadChatSchema()

program_schema_name_only = ProgramSchema(only=('name',))
program_schema_without_program = ProgramSchema(only=('name', 'programNumber'))
programs_schema_without_program = ProgramSchema(only=('name', 'programNumber'), many=True)

program_code_schema = ProgramCodeSchema()
program_code_visual_only_schema = ProgramCodeSchema(only=('visual',))

@app.route('/voice-assistant/chat', methods=['POST'])
def create_chat():
    error = upload_chat_schema.validate(request.json)
    if error:
        return error, 400
    chat = Chat(request.json.get('topic'), request.json.get('personalityId'))
    chat.chatId = str(uuid.uuid4())
    db.session.add(chat)
    db.session.commit()
    returnChat = Chat.query.filter(Chat.chatId == chat.chatId).first_or_404()
    try:
        return jsonify(chat_schema.dump(returnChat)), 201
    except:
        abort(500)

@app.route('/voice-assistant/chat')
def get_all_chats():
    all_chats = Chat.query.all()
    try:
        return jsonify({"voiceAssistantChats": chats_schema.dump(all_chats)})
    except:
        abort(500)

@app.route('/voice-assistant/chat/<string:uuid>', methods=['GET'])
def get_chat_by_id(uuid):
    getChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    try:
        return chat_schema.dump(getChat)
    except:
        abort(500)

@app.route('/voice-assistant/chat/<string:uuid>', methods=['PUT'])
def update_chat(uuid):
    error = upload_chat_schema.validate(request.json)
    if error:
        return error, 400
    chat = Chat(request.json.get('topic'), request.json.get('personalityId'))
    updateChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    updateChat.topic = chat.topic
    updateChat.personalityId = chat.personalityId
    db.session.add(updateChat)
    db.session.commit()
    updateChat = Chat.query.filter(Chat.chatId == updateChat.chatId).first_or_404()
    try:
        return chat_schema.dump(updateChat)
    except:
        abort(500)

@app.route('/voice-assistant/chat/<string:uuid>', methods=['DELETE'])
def delete_chat(uuid):
    deleteChat = Chat.query.filter(Chat.chatId == uuid).first_or_404()
    db.session.delete(deleteChat)
    db.session.commit()
    try:
        return '', 204
    except:
        abort(500)

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
    error = upload_personality_schema.validate(request.json)
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
    error = upload_personality_schema.validate(request.json)
    if error:
        return error, 400
    personality = Personality(request.json.get('name'), uuid, request.json.get('gender'), request.json.get('description'), request.json.get('pauseThreshold'))
    updatePersonality = Personality.query.filter(Personality.personalityId == uuid).first_or_404()
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
    try:
        return '', 204
    except:
        abort(500)


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
    newCameraSettings = CameraSettings(request.json.get('resolution'), request.json.get('refreshRate'), request.json.get('qualityFactor'), request.json.get('resX'), request.json.get('resY'))
    updateCameraSettings = CameraSettings.query.filter(CameraSettings.id == 1).first_or_404()
    updateCameraSettings.resolution = newCameraSettings.resolution
    if newCameraSettings.refreshRate > 1:
        updateCameraSettings.refreshRate = 1
    elif newCameraSettings.refreshRate < 0.1:
        updateCameraSettings.refreshRate = 0.1
    else:
        updateCameraSettings.refreshRate = newCameraSettings.refreshRate
    if newCameraSettings.qualityFactor > 90:
        updateCameraSettings.qualityFactor = 90
    elif newCameraSettings.qualityFactor < 10:
        updateCameraSettings.qualityFactor = 10
    else:
        updateCameraSettings.qualityFactor = newCameraSettings.qualityFactor
    updateCameraSettings.resX = newCameraSettings.resX
    updateCameraSettings.resY = newCameraSettings.resY
    db.session.add(updateCameraSettings)
    db.session.commit()
    response = CameraSettings.query.filter(CameraSettings.id == 1).first_or_404()
    try:
        return camera_settings_schema.dump(response)
    except:
        abort(500)


@app.route('/motor-settings/<string:name>', methods=['GET'])
def get_motor(name):
    motor = Motor.query.filter(Motor.name == name).first_or_404()
    try:
        return motor_schema.dump(motor)
    except:
        abort(500)


@app.route('/motor-settings', methods=['GET'])
def get_motors():
    motors = Motor.query.all()
    try:
        return jsonify({"motorSettings": motors_schema.dump(motors)})
    except:
        abort(500)


@app.route('/motor-settings', methods=['PUT'])
def update_motor():
    error = motor_schema.validate(request.json)
    if error:
        return error, 400
    updateMotor = Motor(request.json.get('name'), request.json.get('pulseWidthMin'), request.json.get('pulseWidthMax'), request.json.get('rotationRangeMin'), request.json.get('rotationRangeMax'), request.json.get('velocity'), request.json.get('acceleration'), request.json.get('deceleration'), request.json.get('period'), request.json.get('turnedOn'))
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
    #motor.effort = updateMotor.effort
    db.session.add(motor)
    db.session.commit()
    try:
        return motor_schema.dump(motor)
    except:
        abort(500)


@app.route('/program', methods=['POST'])
def create_program():
    error = program_schema_name_only.validate(request.json)
    if error:
        return error, 400
    created = Program(request.json.get("name"))
    db.session.add(created)
    db.session.commit()
    create_python_program(created.programNumber)
    try:
        return program_schema_without_program.dump(created)
    except:
        abort(500)

@app.route('/program', methods=['GET'])
def get_all_programs():
    allPrograms = Program.query.all()
    try:
        return jsonify({"programs": programs_schema_without_program.dump(allPrograms)})
    except:
        abort(500)

@app.route('/program/<string:programNumber>', methods=['GET'])
def get_program_by_number(programNumber):
    program = Program.query.filter(Program.programNumber == programNumber).first_or_404()
    try:
        return program_schema_without_program.dump(program)
    except:
        abort(500)

@app.route('/program/<string:programNumber>', methods=['PUT'])
def update_program_by_number(programNumber):
    try:
        data = program_schema_name_only.load(request.json)
    except ValidationError as error:
        return error.messages, 400
    program = Program.query.filter(Program.programNumber == programNumber).first_or_404()
    program.name = data["name"]
    db.session.add(program)
    db.session.commit()
    try:
        return program_schema_without_program.dump(program)
    except:
        abort(500)

@app.route('/program/<string:programNumber>', methods=['DELETE'])
def delete_program_by_number(programNumber):
    delete_python_program(programNumber)
    deleteProgram = Program.query.filter(Program.programNumber == programNumber).first_or_404()
    try:
        db.session.delete(deleteProgram)
        db.session.commit()
        return '', 204
    except:
        abort(500)

@app.route('/program/<string:programNumber>/code', methods=['GET'])
def get_program_code_by_number(programNumber):
    program = Program.query.filter(Program.programNumber == programNumber).first_or_404()
    return program_code_visual_only_schema.dump({
        "visual": program.codeVisual
    })

@app.route('/program/<string:programNumber>/code', methods=['PUT'])
def update_program_code_by_number(programNumber):
    try:
        data = program_code_schema.load(request.json)
    except ValidationError as error:
        return error.messages, 400
    program = Program.query.filter(Program.programNumber == programNumber).first_or_404()
    program.codeVisual = data["visual"]
    db.session.commit()
    update_python_program(programNumber, data["python"])
    return program_code_visual_only_schema.dump(data)

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error':"Entity not found. Please check your path parameter."}), 404

@app.errorhandler(500)
def not_found(error):
    return jsonify({'error': "Internal Server Error, please try later again."}), 500

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")
