from app.app import db


class Motor(db.Model):
    __tablename__ = "motor"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False, unique=True)
    pulse_width_min = db.Column(db.Integer, nullable=False)
    pulse_width_max = db.Column(db.Integer, nullable=False)
    rotation_range_min = db.Column(db.Integer, nullable=False)
    rotation_range_max = db.Column(db.Integer, nullable=False)
    velocity = db.Column(db.Integer, nullable=False)
    acceleration = db.Column(db.Integer, nullable=False)
    deceleration = db.Column(db.Integer, nullable=False)
    period = db.Column(db.Integer, nullable=False)
    turned_on = db.Column(db.Boolean, nullable=False)
    visible = db.Column(db.Boolean, nullable=False)
    invert = db.Column(db.Boolean, nullable=False)
    controller_id = db.Column(db.Integer, db.ForeignKey("controller.id"), nullable=True)
    channel = db.Column(db.Integer, nullable=True)
    current_limit = db.Column(db.Float, nullable=True)
    torque_limit = db.Column(db.Float, nullable=True)

    controller = db.relationship("Controller", back_populates="motors", lazy="joined")
