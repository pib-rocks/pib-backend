from flask import Blueprint, jsonify
import os

bp = Blueprint("ip_controller", __name__)


@bp.route("", methods=["GET"])
def get_host_ip():
    ip_file = "/app/host_ip.txt"
    if os.path.exists(ip_file):
        with open(ip_file, "r") as f:
            ip = f.read().strip()
            return jsonify({"host_ip": ip}), 200
    else:
        return jsonify({"host_ip": ""}), 200
