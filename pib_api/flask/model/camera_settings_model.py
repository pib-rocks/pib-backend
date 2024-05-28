from app.app import db


class CameraSettings(db.Model):
    __tablename__ = "cameraSettings"

    id = db.Column(db.Integer, primary_key=True)
    resolution = db.Column(db.String(3), nullable=False)
    refresh_rate = db.Column(db.Float, nullable=False)
    quality_factor = db.Column(db.Integer, nullable=False)
    res_x = db.Column(db.Integer, nullable=False)
    res_y = db.Column(db.Integer, nullable=False)
