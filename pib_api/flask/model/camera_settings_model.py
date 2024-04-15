from app.app import db


class CameraSettings(db.Model):
    __tablename__ = "cameraSettings"

    id = db.Column(db.Integer, primary_key=True)
    resolution = db.Column(db.String(3), nullable=False)
    refresh_rate = db.Column("refreshRate", db.Float, nullable=False)
    quality_factor = db.Column("qualityFactor", db.Integer, nullable=False)
    res_x = db.Column("resX", db.Integer, nullable=False)
    res_y = db.Column("resY", db.Integer, nullable=False)
