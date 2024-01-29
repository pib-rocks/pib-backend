from app.app import db
from model.bricklet_model import Bricklet

class BrickletPin(db.Model):

    __tablename__ = "brickletPin"
    
    id = db.Column(db.Integer, primary_key=True)
    motorId = db.Column(db.Integer, db.ForeignKey('motor.id'), nullable=False)
    brickletId = db.Column(db.Integer, db.ForeignKey('bricklet.id'), nullable=False)
    pin = db.Column(db.Integer, nullable=False)