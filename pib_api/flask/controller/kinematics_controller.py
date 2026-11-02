from flask import Blueprint, jsonify, request
from marshmallow import ValidationError
from schema.kinematics_schema import inverse_request_schema
from service import kinematics_service

bp = Blueprint("kinematics_controller", __name__)


@bp.route("/inverse", methods=["POST"])
def inverse():
    """Joint angles for one end-effector target.

    A target the arm cannot reach is an ordinary answer (``reach.ok = false`` with the distance
    to the closest reachable point), not an error status: the console keeps dragging.
    """
    payload = request.get_json(silent=True) or {}
    try:
        data = inverse_request_schema.load(payload)
    except ValidationError as error:
        return jsonify({"error": "invalid request", "details": error.messages}), 400

    try:
        answer = kinematics_service.inverse(
            side=data["side"],
            xyz_m=data["xyzM"],
            rpy_rad=data["rpyRad"],
            seed_rad=data["seedRad"],
        )
    except kinematics_service.KinematicsRequestError as error:
        return jsonify({"error": str(error)}), 400
    except ModuleNotFoundError:
        return jsonify({"error": "pib-sdk is not installed in this backend"}), 503

    return jsonify(answer)
