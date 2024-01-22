from schema.bricklet_schema import bricklets_uid_only_schema
from service import bricklet_service
from app.app import db
from flask import jsonify, request, abort


def get_all_bricklets():
    bricklets = bricklet_service.get_all_bricklets()
    try: return jsonify({"bricklets": bricklets_uid_only_schema.dump(bricklets)})
    except: abort(500)