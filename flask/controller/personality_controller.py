from model.personality_model import Personality
from schema.personality_schema import personality_schema, personalities_schema, upload_personality_schema
from app.app import db
import uuid
from flask import abort, jsonify, request

def get_all_personalities():
    all_personalities = Personality.query.all()
    try:
        return jsonify({"voiceAssistantPersonalities": personalities_schema.dump(all_personalities)})
    except:
        abort(500)


def get_personality_by_id(uuid):
    personality = Personality.query.filter(Personality.personalityId == uuid).first_or_404()
    try:
        return personality_schema.dump(personality)
    except:
        abort(500)


def create_personality():
    error = upload_personality_schema.validate(request.json)
    if error:
        return error, 400
    personality = Personality(request.json.get('name'), request.json.get('gender'), request.json.get('pauseThreshold'))
    personality.personalityId = str(uuid.uuid4())
    db.session.add(personality)
    db.session.commit()
    return_personality = Personality.query.filter(Personality.personalityId == personality.personalityId).first_or_404()
    try:
        return jsonify(personality_schema.dump(return_personality)), 201
    except:
        abort(500)


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


def delete_personality(uuid):
    personality = Personality.query.filter(Personality.personalityId == uuid).first_or_404()
    db.session.delete(personality)
    db.session.commit()
    try:
        return '', 204
    except:
        abort(500)