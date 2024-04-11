from app.app import db
from model.bricklet_pin_model import BrickletPin
from model.util import generate_uuid


class Bricklet(db.Model):
    __tablename__ = "bricklet"

    id = db.Column(db.Integer, primary_key=True)
    uid = db.Column(db.String(30), nullable=False, default=generate_uuid, unique=True)
    bricklet_number = db.Column("brickletNumber", db.Integer, nullable=False, unique=True)
    bricklet_pins = db.relationship(BrickletPin, backref="bricklet", lazy=True, cascade="all,delete-orphan")
