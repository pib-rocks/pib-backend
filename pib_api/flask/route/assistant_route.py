from flask import Blueprint
from flask.controller import assistant_controler

blueprint = Blueprint('blueprint', __name__)

blueprint.route('', methods=['GET'])(assistant_controler.get_all_assistants_settings)