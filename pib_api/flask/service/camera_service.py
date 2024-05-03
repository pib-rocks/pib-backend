from model.camera_settings_model import CameraSettings
from typing import Any
from app.app import db
import sys

def get_camera_settings() -> CameraSettings:
    return CameraSettings.query.one()


def update_camera_settings(camera_settings_dto: dict[str, Any]) -> CameraSettings:
    print(camera_settings_dto, file=sys.stderr)
    camera_settings = get_camera_settings()
    camera_settings.resolution = camera_settings_dto["resolution"]
    camera_settings.refreshRate = camera_settings_dto["refreshRate"]
    camera_settings.qualityFactor = camera_settings_dto["qualityFactor"]
    camera_settings.resX = camera_settings_dto["resX"]
    camera_settings.resY = camera_settings_dto["resY"]
    db.session.flush()
    return camera_settings