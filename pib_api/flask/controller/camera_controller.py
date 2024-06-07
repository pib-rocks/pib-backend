from schema.camera_settings_schema import camera_settings_schema
from service import camera_service
from flask import abort, request
from app.app import db


bp = Blueprint("camera_controller", __name__)


@bp.route("", methods=["GET", "POST"])
def get_camera_settings():
    camera_settings = camera_service.get_camera_settings()
    try:
        return camera_settings_schema.dump(camera_settings)
    except Exception:
        abort(500)


@bp.route("", methods=["PUT"])
def update_camera_settings():
    camera_settings_dto = camera_settings_schema.load(request.json)
    camera_settings = camera_service.update_camera_settings(camera_settings_dto)
    db.session.commit()
    try:
        return camera_settings_schema.dump(camera_settings)
    except Exception:
        abort(500)
