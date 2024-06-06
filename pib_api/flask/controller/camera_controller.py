from model.camera_settings_model import CameraSettings
from schema.camera_settings_schema import camera_settings_schema
from app.app import db
from flask import abort, request, Blueprint

bp = Blueprint("camera_controller", __name__)


@bp.route("", methods=["GET", "POST"])
def get_camera_settings():
    cameraSettings = CameraSettings.query.all()
    try:
        return camera_settings_schema.dump(cameraSettings[0])
    except:
        abort(500)


@bp.route("", methods=["PUT"])
def update_camera_settings():
    error = camera_settings_schema.validate(request.json)
    if error:
        return error, 400
    newCameraSettings = CameraSettings(
        request.json.get("resolution"),
        request.json.get("refresh_rate"),
        request.json.get("quality_factor"),
        request.json.get("res_x"),
        request.json.get("res_y"),
    )
    updateCameraSettings = CameraSettings.query.filter(
        CameraSettings.id == 1
    ).first_or_404()
    updateCameraSettings.resolution = newCameraSettings.resolution
    updateCameraSettings.refresh_rate = min(max(0.1, newCameraSettings.refresh_rate), 1)
    updateCameraSettings.quality_factor = min(
        max(10, newCameraSettings.quality_factor), 90
    )
    updateCameraSettings.resX = newCameraSettings.resX
    updateCameraSettings.resY = newCameraSettings.resY
    db.session.add(updateCameraSettings)
    db.session.commit()
    response = CameraSettings.query.filter(CameraSettings.id == 1).first_or_404()
    try:
        return camera_settings_schema.dump(response)
    except:
        abort(500)
