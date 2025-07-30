from pib_api.flask.app.app import db


class MotorPosition(db.Model):

    __tablename__ = "motor_position"

    id = db.Column(db.Integer, primary_key=True)
    position = db.Column(db.Integer, nullable=False)
    motor_name = db.Column(db.Integer, db.ForeignKey("motor.name"), nullable=False)
    pose_id = db.Column(db.Integer, db.ForeignKey("pose.id"), nullable=False)
