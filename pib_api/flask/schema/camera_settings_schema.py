from model.camera_settings_model import CameraSettings
from app.app import ma

class CameraSettingsSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)

camera_settings_schema = CameraSettingsSchema()