from pib_api.flask.app.app import db
from model.util import generate_uuid


class Pose(db.Model):

    __tablename__ = "pose"

    id = db.Column(db.Integer, primary_key=True)
    pose_id = db.Column(
        db.String(255), nullable=False, default=generate_uuid, unique=True
    )
    name = db.Column(db.String(255), nullable=False, unique=True)
    motor_positions = db.relationship(
        "MotorPosition", backref="pose", lazy=True, cascade="all,, delete-orphan"
    )
