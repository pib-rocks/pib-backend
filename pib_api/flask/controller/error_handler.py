import traceback

from app.app import app
from flask import jsonify


class ChatMessageContentTooLongError(Exception):
    """POST body content exceeds the chat message column limit (PR-2021)."""

    def __init__(self, field: str, limit: int, received_length: int):
        self.field = field
        self.limit = limit
        self.received_length = received_length
        super().__init__(
            f"Message content exceeds the maximum length of {limit} characters."
        )


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


def handle_chat_message_content_too_long_error(error):
    app.logger.warning(error)
    return (
        jsonify(
            {
                "error": str(error),
                "field": error.field,
                "limit": error.limit,
                "receivedLength": error.received_length,
            }
        ),
        413,
    )


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


def handle_conflict_error(error):
    """A request the data model refuses - report the refusal, not a server fault.

    The exception's own message is the answer on purpose: it names the entity that
    cannot be changed in this state, which a generic "Bad request." would hide
    (PR-1974).
    """
    app.logger.warning(error)
    return jsonify({"error": str(error)}), 409


def handle_invalid_request_error(error):
    """The request does not describe a valid change - the reason is the answer.

    Distinct from handle_bad_request_error's generic "Bad request." on purpose: a
    caller sending a motor list that does not match the stored pose should learn which
    part was wrong (PR-1974).
    """
    app.logger.warning(error)
    return jsonify({"error": str(error)}), 400


def handle_unknown_error(error):
    app.logger.error(traceback.format_exc())
    app.logger.error(error)
    return jsonify({"error": "an unknown error occured."}), 500
