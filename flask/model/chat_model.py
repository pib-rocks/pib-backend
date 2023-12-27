from app.app import db
import uuid

class Chat(db.Model):

    __tablename__ = "chat"
    id = db.Column(db.Integer, primary_key=True)
    chatId= db.Column(db.String(255), nullable=False, unique=True)
    topic = db.Column(db.String(255), nullable=False)
    personalityId = db.Column(db.String(255), db.ForeignKey('personality.personalityId'), nullable=False)
    messages = db.relationship('ChatMessage', backref='chat', lazy=True, cascade="all,delete")
    
    def __init__(self, topic, personality_id):
        self.chatId = str(uuid.uuid4())
        self.topic = topic
        self.personalityId = personality_id