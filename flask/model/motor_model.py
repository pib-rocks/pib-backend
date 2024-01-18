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
    brickletPins = db.relationship('BrickletPin', backref='motor', lazy=True, cascade="all,delete")
    
    def __init__(self, **kwargs):
        self.name = kwargs['name']
        self.pulseWidthMin = kwargs['pulseWidthMin']
        self.pulseWidthMax = kwargs['pulseWidthMax']
        self.rotationRangeMin = kwargs['rotationRangeMin']
        self.rotationRangeMax = kwargs['rotationRangeMax']
        self.velocity = kwargs['velocity']
        self.acceleration = kwargs['acceleration']
        self.deceleration = kwargs['deceleration']
        self.period = kwargs['period']
        self.turnedOn = kwargs['turnedOn']
        self.visible = kwargs['visible']