import os
from flask import Blueprint, jsonify
from app.app import app

bp = Blueprint("ip_controller", __name__)


@bp.route("", methods=["GET"])
def get_host_ip():
    ip_file = app.config.get("HOST_IP_FILE", "host_ip.txt")
    if os.path.exists(ip_file):
        with open(ip_file, "r") as f:
            ip = f.read().strip()
            return jsonify({"host_ip": ip}), 200
    else:
        return jsonify({"host_ip": ""}), 200
