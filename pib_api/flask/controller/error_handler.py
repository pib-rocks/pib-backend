from flask import jsonify
from app.app import app


def handle_not_found_error(error):
    app.logger.error(error)
    return (
        jsonify({"error": "Entity not found. Please check your path parameter."}),
        404,
    )


def handle_internal_server_error(error):
    app.logger.error(traceback.format_exc())
    app.logger.error(error)
    return jsonify({"error": "Internal Server Error, please try later again."}), 500


def handle_not_implemented_error(error):
    app.logger.error(error)
    return jsonify({"error": "Not implemented."}), 501


def handle_bad_request_error(error):
    app.logger.error(error)
    return jsonify({"error": "Bad request."}), 400


def handle_unknown_error(error):
    app.logger.error(traceback.format_exc())
    app.logger.error(error)
    return jsonify({"error": "an unknown error occured."}), 500
