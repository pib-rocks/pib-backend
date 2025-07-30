from pib_api.flask.app.app import db
from model.util import generate_uuid
from sqlalchemy.sql import func


class ChatMessage(db.Model):
    __tablename__ = "chatMessage"

    id = db.Column(db.Integer, primary_key=True)
    message_id = db.Column(
        db.String(255), default=generate_uuid, nullable=False, unique=True
    )
    timestamp = db.Column(db.DateTime, nullable=False, default=func.now())
    is_user = db.Column(db.Boolean, nullable=False)
    content = db.Column(db.String(100000), nullable=False)
    chat_id = db.Column(db.String(255), db.ForeignKey("chat.chat_id"), nullable=False)
