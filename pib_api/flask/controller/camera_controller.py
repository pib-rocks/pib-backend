from model.camera_settings_model import CameraSettings
from schema.camera_settings_schema import camera_settings_schema
from app.app import db
from flask import abort, request


def get_camera_settings():
    cameraSettings = CameraSettings.query.all()
    try:
        return camera_settings_schema.dump(cameraSettings[0])
    except:
        abort(500)


def update_camera_settings():
    error = camera_settings_schema.validate(request.json)
    if error:
        return error, 400
    newCameraSettings = CameraSettings(
        request.json.get("resolution"),
        request.json.get("refreshRate"),
        request.json.get("qualityFactor"),
        request.json.get("resX"),
        request.json.get("resY"),
    )
    updateCameraSettings = CameraSettings.query.filter(
        CameraSettings.id == 1
    ).first_or_404()
    updateCameraSettings.resolution = newCameraSettings.resolution
    updateCameraSettings.refreshRate = min(max(0.1, newCameraSettings.refreshRate), 1)
    updateCameraSettings.qualityFactor = min(
        max(10, newCameraSettings.qualityFactor), 90
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
