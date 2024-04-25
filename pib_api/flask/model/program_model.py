from app.app import db
import uuid

from model.util import generate_uuid


class Program(db.Model):
    __tablename__ = "program"
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False, unique=True)
    code_visual = db.Column(db.String(100000), nullable=False, default="{}")
    program_number = db.Column(db.String(50), unique=True, nullable=False, default=generate_uuid)

    def __init__(self, name, code_visual: str = "{}"):
        self.name = name
        self.code_visual = code_visual
