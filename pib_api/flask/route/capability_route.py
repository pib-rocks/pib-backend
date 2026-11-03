from flask import Blueprint
from controller import capability_controller

blueprint = Blueprint("capability_route", __name__)

blueprint.route("", methods=["GET"])(capability_controller.get_capabilities)
