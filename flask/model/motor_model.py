from app.app import db

class Motor(db.Model):
    
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
    active = db.Column(db.Boolean, nullable=False)
    effort = db.Column(db.Integer, nullable=True)
    
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
        self.active = args[10]
        if len(args) > 11:
            self.effort = args[11]