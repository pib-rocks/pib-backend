import logging

from flask import abort, Blueprint, make_response

from schema.assistant_schema import assistants_schema
from service import assistant_service

bp = Blueprint('assistant_controller', __name__)


@bp.route('/', methods=['GET'])
def get_all_assistants_settings():
    try:
        assistants = assistant_service.get_all_assistants()
        assistant_dtos = assistants_schema.dump(assistants)
    except Exception as e:
        logging.error(e)
        abort(500, description="Internal Server Error")

    return make_response(assistant_dtos, 200)
