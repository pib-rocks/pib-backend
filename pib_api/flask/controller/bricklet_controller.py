from schema.bricklet_schema import (
    bricklet_uid_only_schema,
    bricklet_schema,
    bricklets_schema,
)
from service import bricklet_service
from flask import request, Blueprint, jsonify

bp = Blueprint("bricklet_controller", __name__)


@bp.route("", methods=["GET"])
def get_all_bricklets():
    bricklets = bricklet_service.get_all_bricklets()
    return jsonify({"bricklets": bricklets_schema.dump(bricklets)})


@bp.route("/<string:bricklet_number>", methods=["GET"])
def get_bricklet(bricklet_number: str):
    bricklet = bricklet_service.get_bricklet(bricklet_number)
    return bricklet_uid_only_schema.dump(bricklet)


@bp.route("/<string:bricklet_number>", methods=["PUT"])
def update_bricklet(bricklet_number: str):
    uid = bricklet_uid_only_schema.load(request.json)["uid"]
    bricklet = bricklet_service.set_bricklet_uid(bricklet_number, uid)
    return bricklet_schema.dump(bricklet)
