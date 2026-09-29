from marshmallow import EXCLUDE
from model.camera_settings_model import CameraSettings
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class CameraSettingsSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    """JSON uses camelCase; load() yields snake_case model field names.

    Extra keys (for example UI-only ``isActive``) are ignored: they are not
    stored on CameraSettings and are not part of GET responses.
    """

    class Meta:
        model = CameraSettings
        exclude = ("id",)
        unknown = EXCLUDE


camera_settings_schema = CameraSettingsSchemaSQLAutoWith()
