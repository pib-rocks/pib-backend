from flask import Blueprint, Response, abort, jsonify, request

from service import (
    bricklet_status_service,
    diagnostics_service,
    docker_service,
    system_info_service,
)

bp = Blueprint("system_controller", __name__)


@bp.route("/info", methods=["GET"])
def get_system_info():
    return jsonify(system_info_service.get_system_info()), 200


@bp.route("/containers", methods=["GET"])
def get_containers():
    try:
        containers = docker_service.list_containers()
    except RuntimeError as exc:
        abort(500, description=str(exc))
    return jsonify({"containers": containers}), 200


@bp.route("/containers/<string:name>/logs", methods=["GET"])
def get_container_logs(name: str):
    tail = request.args.get("tail", default=200, type=int)
    try:
        payload = docker_service.get_container_logs(name, tail=tail)
    except LookupError as exc:
        abort(404, description=str(exc))
    except RuntimeError as exc:
        abort(500, description=str(exc))
    return jsonify(payload), 200


@bp.route("/bricklets/status", methods=["GET"])
def get_bricklets_status():
    return jsonify(bricklet_status_service.get_bricklets_status()), 200


@bp.route("/diagnostics.zip", methods=["GET"])
def download_diagnostics():
    try:
        content = diagnostics_service.build_diagnostics_zip()
    except Exception as exc:
        abort(500, description=f"Unable to build diagnostics archive: {exc}")
    filename = diagnostics_service.diagnostics_filename()
    return Response(
        content,
        mimetype="application/zip",
        headers={
            "Content-Disposition": f'attachment; filename="{filename}"',
        },
    )
