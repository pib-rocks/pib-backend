import os
from os.path import dirname, abspath
from flask.wrappers import Response

from flask import Flask
from flask_cors import CORS
from flask_marshmallow import Marshmallow
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy

# BASE_DIR should be the working directory of the app, e.g. 'flask/'
BASE_DIR = dirname(dirname(abspath(__file__)))  # not used yet

app = Flask(__name__)
app.config.from_object("config.Config")
db = SQLAlchemy(app)
ma = Marshmallow(app)
migrate = Migrate(app, db)
CORS(app)

if not os.path.exists(app.config.get("PYTHON_CODE_DIR")):
    os.makedirs(app.config.get("PYTHON_CODE_DIR"))

# Imported at the bottom to prevent a circular import error
from commands import seed_db

app.cli.add_command(seed_db)


@app.after_request
def handle_session_after_request(response: Response) -> Response:
    if 200 <= response.status_code < 300:
        db.session.commit()
    else:
        db.session.rollback()
    return response
