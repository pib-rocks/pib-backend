from app.app import db

class AssistantModel(db.Model):
    
    __tablename__ = "assistant_model"

    id = db.Column(db.Integer, primary_key=True)
    api_name= db.Column(db.String(255), nullable=False, unique=True)
    visual_name = db.Column(db.String(255), nullable=False, unique=True)