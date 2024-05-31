from app.app import db


class BrickletPin(db.Model):
    __tablename__ = "brickletPin"

    id = db.Column(db.Integer, primary_key=True)
    motor_id = db.Column(db.Integer, db.ForeignKey("motor.id"), nullable=False)
    bricklet_id = db.Column(db.Integer, db.ForeignKey("bricklet.id"), nullable=False)
    pin = db.Column(db.Integer, nullable=False)
    invert = db.Column(db.Boolean, nullable=False)
