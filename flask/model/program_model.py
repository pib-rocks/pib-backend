from app.app import db
import uuid

class Program(db.Model):
    
    __tablename__ = "program"
    id = db.Column(db.Integer, primary_key=True)
    name= db.Column(db.String(255), nullable=False, unique=True)
    codeVisual = db.Column(db.String(100000), nullable=False)
    programNumber = db.Column(db.String(50), nullable=False)

    def __init__(self, name, code_visual = "{}"):
        self.name = name
        self.codeVisual = code_visual
        self.programNumber = str(uuid.uuid4())