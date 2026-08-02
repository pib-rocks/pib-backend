from flask import Blueprint, jsonify, request
from service import docker_admin_service

bp = Blueprint("docker_controller", __name__)


@bp.route("/containers", methods=["GET"])
def get_containers():
    containers = docker_admin_service.get_containers()
    return jsonify(containers), 200


@bp.route("/containers/<name>/start", methods=["POST"])
def start_container(name):
    status_code, res = docker_admin_service.start_container(name)
    return jsonify(res), status_code


@bp.route("/containers/<name>/stop", methods=["POST"])
def stop_container(name):
    status_code, res = docker_admin_service.stop_container(name)
    return jsonify(res), status_code


@bp.route("/containers/<name>/restart", methods=["POST"])
def restart_container(name):
    status_code, res = docker_admin_service.restart_container(name)
    return jsonify(res), status_code


@bp.route("/containers/<name>/logs", methods=["GET"])
def get_container_logs(name):
    tail = request.args.get("tail", default=500, type=int)
    status_code, res = docker_admin_service.get_container_logs(name, tail=tail)
    return jsonify(res), status_code


@bp.route("/containers/<name>/clear-logs", methods=["POST"])
def clear_container_logs(name):
    status_code, res = docker_admin_service.clear_container_logs(name)
    return jsonify(res), status_code


@bp.route("/admin/purge", methods=["POST"])
def purge_docker():
    status_code, res = docker_admin_service.purge_docker()
    return jsonify(res), status_code
