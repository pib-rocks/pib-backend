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
    personality_id = db.Column(db.String(255), nullable=False)
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pause_threshold = db.Column(db.Numeric, nullable=False)
    def __init__(self, *args):
        if len(args) == 3:
            self.eq3(args)
        if len(args) == 5:
            self.eq5(args)

    def eq3(self, args): 
        self.name = args[0]
        self.personality_id = str(uuid.uuid4())
        self.description = ""
        self.gender = args[1]
        self.pause_threshold = args[2]

    def eq5(self, args): 
        self.name = args[0]
        self.personality_id = args[1]
        self.gender = args[2]
        self.description = args[3]
        self.pause_threshold = args[4]

class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id',)

personality_schema = PersonalitySchema()
personalities_schema = PersonalitySchema(many=True)

@app.route('/voice-assistant/personality')
def get_all_personalities():
    all_personalities = Personality.query.all()
    return jsonify({"voiceAssistantPersonalities": personalities_schema.dump(all_personalities)})

@app.route('/voice-assistant/personality/<string:uuid>', methods=['GET'])
def get_personality_by_id(uuid):
    getPersonality = Personality.query.filter(Personality.personality_id == uuid).first()
    return personality_schema.jsonify(getPersonality)

@app.route('/voice-assistant/personality', methods=['POST'])
def create_personality():
    personality = Personality(request.json.get('name'), request.json.get('gender'), request.json.get('pauseThreshold'))
    db.session.add(personality)
    db.session.commit()
    returnPersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    return jsonify(personality_schema.dump(returnPersonality)), 201

@app.route('/voice-assistant/personality/<string:uuid>', methods=['PUT'])
def update_personality(uuid):
    personality = Personality(request.json.get('name'), uuid, request.json.get('gender'), request.json.get('description'), request.json.get('pauseThreshold'))
    updatePersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    updatePersonality.name = personality.name
    updatePersonality.description = personality.description
    updatePersonality.gender = personality.gender
    updatePersonality.pause_threshold = personality.pause_threshold
    db.session.add(updatePersonality)
    db.session.commit()
    updatePersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    return personality_schema.jsonify(updatePersonality)

@app.route('/voice-assistant/personality/<string:uuid>', methods=['DELETE'])
def delete_personality(uuid):
    deletePersonality = Personality.query.filter(Personality.personality_id == uuid).first()
    db.session.delete(deletePersonality)
    db.session.commit()
    return '', 204

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")