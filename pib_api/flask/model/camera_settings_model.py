from app.app import db


class CameraSettings(db.Model):

    __tablename__ = "cameraSettings"
    id = db.Column(db.Integer, primary_key=True)
    resolution = db.Column(db.String(3), nullable=False)
    refreshRate = db.Column(db.Float, nullable=False)
    qualityFactor = db.Column(db.Integer, nullable=False)
    resX = db.Column(db.Integer, nullable=False)
    resY = db.Column(db.Integer, nullable=False)

    def __init__(self, resolution, refresh_rate, quality_factor, res_x, res_y):
        self.resolution = resolution
        self.refreshRate = refresh_rate
        self.qualityFactor = quality_factor
        self.resX = res_x
        self.resY = res_y
