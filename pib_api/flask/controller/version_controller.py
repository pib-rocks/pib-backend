from flask import Blueprint, jsonify

bp = Blueprint("version_controller", __name__)


def _read_app_version():
    try:
        import version as version_module

        value = getattr(version_module, "APP_VERSION", None)
        if isinstance(value, str) and value.strip():
            return value.strip()
    except Exception:
        pass

    try:
        with open("/app/version.py", encoding="utf-8") as version_file:
            for line in version_file:
                stripped = line.strip()
                if stripped.startswith("APP_VERSION="):
                    raw = stripped.split("=", 1)[1].strip().strip('"').strip("'")
                    if raw:
                        return raw
                    break
    except Exception:
        pass

    return "unknown"


@bp.route("", methods=["GET"])
def get_version():
    return jsonify({"version": _read_app_version()}), 200
