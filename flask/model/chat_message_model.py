from app.app import db
import uuid

class ChatMessage(db.Model):

    __tablename__ = "chatMessage"
    id = db.Column(db.Integer, primary_key=True) 
    messageId = db.Column(db.String(255), nullable=False, unique=True) 
    timestamp = db.Column(db.String(255), nullable=False)
    isUser = db.Column(db.Boolean, nullable=False) 
    content = db.Column(db.String(255), nullable=False)
    chatId = db.Column(db.String(255), nullable=False, unique=True)
    
    def __init__(self, isUser, content, chatId):
        self.messageId = str(uuid.uuid4())
        self.timestamp = "TODO"
        self.isUser = isUser
        self.content = content
        self.chatId = chatId