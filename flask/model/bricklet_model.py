from app.app import db

class Bricklet(db.Model):

    __tablename__ = "bricklet"
    
    id = db.Column(db.Integer, primary_key=True)
    uid = db.Column(db.String(30), nullable=False, unique=True)
    brickletPins = db.relationship('BrickletPin', backref='bricklet', lazy=True, cascade="all,delete-orphan")
