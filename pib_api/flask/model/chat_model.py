import uuid

from app.app import db
from model.chat_message_model import ChatMessage
from model.util import generate_uuid


class Chat(db.Model):
    __tablename__ = "chat"

    id = db.Column(db.Integer, primary_key=True)
    chat_id = db.Column("chatId", db.String(255), nullable=False, default=generate_uuid, unique=True)
    topic = db.Column(db.String(255), nullable=False)
    personality_id = db.Column("personalityId", db.String(255), db.ForeignKey("personality.personalityId"), nullable=False)
    messages = db.relationship("ChatMessage", backref="chat", lazy=True, cascade="all,delete",
                               order_by=ChatMessage.timestamp.desc())
