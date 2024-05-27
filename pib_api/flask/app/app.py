import os
from os.path import dirname, abspath

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
