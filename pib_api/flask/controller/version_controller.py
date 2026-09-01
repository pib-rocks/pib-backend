from flask import Blueprint, jsonify

bp = Blueprint("version_controller", __name__)


VERSION_FILES = (
    "/etc/pib_version",
    "/app/version.py",
)  # /etc survives the /app volume mount


def _read_app_version():
    for path in VERSION_FILES:
        try:
            with open(path, encoding="utf-8") as vf:
                value = vf.read().strip().strip('"').strip("'")
            if value:
                return value
        except Exception:
            continue
    return "unknown"


@bp.route("", methods=["GET"])
def get_version():
    return jsonify({"version": _read_app_version()}), 200
