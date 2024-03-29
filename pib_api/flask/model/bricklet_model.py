from app.app import db
from model.bricklet_pin_model import BrickletPin

class Bricklet(db.Model):

    __tablename__ = "bricklet"
    
    id = db.Column(db.Integer, primary_key=True)
    uid = db.Column(db.String(30), nullable=False, unique=True)
    brickletNumber = db.Column(db.Integer, nullable=False, unique=True)
    brickletPins = db.relationship(BrickletPin, backref='bricklet', lazy=True, cascade="all,delete-orphan")