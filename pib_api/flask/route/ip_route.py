from flask import Blueprint
from controller import ip_controller

blueprint = Blueprint("blueprint", __name__)

blueprint.route("", methods=["GET"])(ip_controller.get_host_ip)
