from app.app import db
from sqlalchemy.sql import func
import uuid

class ChatMessage(db.Model):

    __tablename__ = "chatMessage"
    id = db.Column(db.Integer, primary_key=True) 
    messageId = db.Column(db.String(255), nullable=False, unique=True) 
    timestamp = db.Column(db.DateTime, nullable=False , default=func.now())
    isUser = db.Column(db.Boolean, nullable=False) 
    content = db.Column(db.String(100000), nullable=False)
    chatId = db.Column(db.String(255), db.ForeignKey('chat.chatId'), nullable=False)
    
    def __init__(self, isUser, content, chatId):
        self.messageId = str(uuid.uuid4())
        self.isUser = isUser
        self.content = content
        self.chatId = chatId