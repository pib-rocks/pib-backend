from app.app import db
import uuid

class Personality(db.Model):

    __tablename__ = 'personality'

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), nullable=False, unique=True)
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pauseThreshold = db.Column(db.Float, nullable=False)
    chats = db.relationship('Chat', backref='personality', lazy=True, cascade="all,delete")
    assistant_id = db.Column(db.Integer, db.ForeignKey('assistant_model.id'), nullable=True)