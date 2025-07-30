from pib_api.flask.app.app import db
from model.bricklet_pin_model import BrickletPin


class Bricklet(db.Model):
    __tablename__ = "bricklet"

    id = db.Column(db.Integer, primary_key=True)
    uid = db.Column(db.String(30), nullable=True, unique=True)
    bricklet_number = db.Column(db.Integer, nullable=False, unique=True)
    type = db.Column(
        db.Enum("Solid State Relay Bricklet", "Servo Bricklet", name="bricklet_type"),
        nullable=False,
    )
    bricklet_pins = db.relationship(
        BrickletPin, backref="bricklet", lazy=True, cascade="all,delete-orphan"
    )
