import traceback

from app.app import app
from flask import jsonify


def handle_not_found_error(error):
    app.logger.error(error)
    return (
        jsonify({"error": "Entity not found. Please check your path parameter."}),
        404,
    )


def handle_internal_server_error(error):
    app.logger.error(traceback.format_exc())
    app.logger.error(error)
    return (
        jsonify(
            {
                "error": getattr(
                    error,
                    "description",
                    "Internal Server Error, please try later again.",
                )
            }
        ),
        500,
    )


def handle_not_implemented_error(error):
    app.logger.error(error)
    return jsonify({"error": "Not implemented."}), 501


def handle_bad_request_error(error):
    app.logger.error(error)
    return jsonify({"error": "Bad request."}), 400


def handle_unprocessable_entity_error(error):
    app.logger.error(error)
    return jsonify({"error": error.description}), 422


def handle_method_not_allowed_error(error):
    app.logger.error(error)
    response = jsonify({"error": "Method not allowed for this endpoint."})
    valid_methods = getattr(error, "valid_methods", None)
    if valid_methods:
        response.headers["Allow"] = ", ".join(valid_methods)
    return response, 405


def handle_unknown_error(error):
    app.logger.error(traceback.format_exc())
    app.logger.error(error)
    return jsonify({"error": "an unknown error occured."}), 500
