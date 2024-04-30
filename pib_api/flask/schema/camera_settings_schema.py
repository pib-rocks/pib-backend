from model.camera_settings_model import CameraSettings
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class CameraSettingsSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)


camera_settings_schema = CameraSettingsSchemaSQLAutoWith()
