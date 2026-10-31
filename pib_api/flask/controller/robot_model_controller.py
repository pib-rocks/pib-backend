from flask import Blueprint, jsonify, request, send_file

from service import robot_model_service as rms

bp = Blueprint("robot_model_controller", __name__)


def _not_installed_response():
    return (
        jsonify(
            {
                "error": "no robot model installed",
                "hint": "run tools/install_robot_model.py",
            }
        ),
        404,
    )


def _version_query_matches() -> bool:
    installed = rms.installed_version()
    if not installed:
        return False
    return request.args.get("v") == installed


def _cache_control_value() -> str:
    if _version_query_matches():
        return "public, max-age=31536000, immutable"
    return "no-cache"


def _send_installed_file(relative: str, mimetype: str | None = None):
    path = rms.file_path(relative)
    if path is None:
        return _not_installed_response()
    if_none_match = request.headers.get("If-None-Match")
    etag = rms.file_etag(path)
    if if_none_match and if_none_match.strip() == etag:
        return "", 304, {"Cache-Control": _cache_control_value(), "ETag": etag}
    response = send_file(path, mimetype=mimetype, conditional=False)
    response.headers["Cache-Control"] = _cache_control_value()
    response.headers["ETag"] = etag
    return response


@bp.route("/manifest.json", methods=["GET"])
def get_manifest():
    manifest = rms.load_manifest()
    if manifest is None:
        return _not_installed_response()
    response = jsonify(manifest)
    version_path = rms.file_path("version.json")
    if version_path:
        response.headers["Cache-Control"] = _cache_control_value()
        etag = rms.file_etag(version_path)
        response.headers["ETag"] = etag
        if_none_match = request.headers.get("If-None-Match")
        if if_none_match and if_none_match.strip() == etag:
            return "", 304, {"Cache-Control": _cache_control_value(), "ETag": etag}
    return response


@bp.route("/model.json", methods=["GET"])
def get_model_json():
    return _send_installed_file("model.json", mimetype="application/json")


@bp.route("/mapping.json", methods=["GET"])
def get_mapping_json():
    return _send_installed_file("mapping.json", mimetype="application/json")


@bp.route("/meshes/<name>", methods=["GET"])
def get_mesh(name: str):
    path = rms.resolve_mesh(name)
    if path is None:
        if not rms.is_installed():
            return _not_installed_response()
        return jsonify({"error": "mesh not found"}), 404
    if_none_match = request.headers.get("If-None-Match")
    etag = rms.file_etag(path)
    if if_none_match and if_none_match.strip() == etag:
        return "", 304, {
            "Cache-Control": _cache_control_value(),
            "ETag": etag,
        }
    response = send_file(path, mimetype="model/gltf-binary", conditional=False)
    response.headers["Cache-Control"] = _cache_control_value()
    response.headers["ETag"] = etag
    return response
