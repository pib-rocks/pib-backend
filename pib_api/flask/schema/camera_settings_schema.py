from model.camera_settings_model import CameraSettings
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class CameraSettingsSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)


camera_settings_schema = CameraSettingsSchemaSQLAutoWith()
