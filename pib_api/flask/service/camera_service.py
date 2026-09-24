from model.camera_settings_model import CameraSettings
from typing import Any
from app.app import db


def get_camera_settings() -> CameraSettings:
    return CameraSettings.query.one()


def update_camera_settings(camera_settings_dto: dict[str, Any]) -> CameraSettings:
    camera_settings = get_camera_settings()
    camera_settings.resolution = camera_settings_dto["resolution"]
    camera_settings.refresh_rate = camera_settings_dto["refresh_rate"]
    camera_settings.quality_factor = camera_settings_dto["quality_factor"]
    camera_settings.res_x = camera_settings_dto["res_x"]
    camera_settings.res_y = camera_settings_dto["res_y"]
    db.session.flush()
    return camera_settings
