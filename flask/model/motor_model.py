from app.app import db
from model.bricklet_pin_model import BrickletPin

class Motor(db.Model):

    __tablename__ = "motor"
    
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False, unique=True)
    pulseWidthMin = db.Column(db.Integer, nullable=False)
    pulseWidthMax = db.Column(db.Integer, nullable=False)
    rotationRangeMin = db.Column(db.Integer, nullable=False)
    rotationRangeMax = db.Column(db.Integer, nullable=False)
    velocity = db.Column(db.Integer, nullable=False)
    acceleration = db.Column(db.Integer, nullable=False)
    deceleration = db.Column(db.Integer, nullable=False)
    period = db.Column(db.Integer, nullable=False)
    turnedOn = db.Column(db.Boolean, nullable=False)
    visible = db.Column(db.Boolean, nullable=False)
    invert = db.Column(db.Boolean, nullable=False)
    
    def __init__(self, *args):
        self.name = args[0]
        self.pulseWidthMin = args[1]
        self.pulseWidthMax = args[2]
        self.rotationRangeMin = args[3]
        self.rotationRangeMax = args[4]
        self.velocity = args[5]
        self.acceleration = args[6]
        self.deceleration = args[7]
        self.period = args[8]
        self.turnedOn = args[9]
        self.visible = args[10]
        self.invert = args[11]