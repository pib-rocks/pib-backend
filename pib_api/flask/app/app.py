import os

from flask import Flask
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy
from flask_marshmallow import Marshmallow
from flask_cors import CORS
from os.path import dirname, abspath, expanduser

# BASE_DIR should be the working directory of the app, e.g. 'flask/'
BASE_DIR = dirname(dirname(abspath(__file__)))
DEFAULT_SQL_URI = f"sqlite:///{os.path.join(BASE_DIR, 'pibdata.db')}"
DEFAULT_PYTHON_CODE_DIR = os.path.join(
    expanduser("~"), "cerebra_programs"
)  # not used yet

app = Flask(__name__)
app.config["SQLALCHEMY_DATABASE_URI"] = os.getenv(
    "SQLALCHEMY_DATABASE_URI", DEFAULT_SQL_URI
)
app.config["PYTHON_CODE_DIR"] = os.getenv("PYTHON_CODE_DIR", DEFAULT_PYTHON_CODE_DIR)
db = SQLAlchemy(app)
ma = Marshmallow(app)
migrate = Migrate(app, db)
CORS(app)

if not os.path.exists(app.config.get("PYTHON_CODE_DIR")):
    os.makedirs(app.config.get("PYTHON_CODE_DIR"))


# Imported at the bottom to prevent a circular import error
from commands import seed_db

app.cli.add_command(seed_db)
