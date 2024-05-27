from app.app import db
from model.chat_message_model import ChatMessage
import uuid


class Chat(db.Model):

    __tablename__ = "chat"

    id = db.Column(db.Integer, primary_key=True)
    chatId = db.Column(db.String(255), nullable=False, unique=True)
    topic = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(
        db.String(255), db.ForeignKey("personality.personalityId"), nullable=False
    )
    messages = db.relationship(
        "ChatMessage",
        backref="chat",
        lazy=True,
        cascade="all,delete",
        order_by=ChatMessage.timestamp.desc(),
    )
