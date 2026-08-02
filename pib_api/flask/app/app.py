import os
import sqlite3
from os.path import dirname, abspath
from flask.wrappers import Response

from flask import Flask
from flask_cors import CORS
from flask_marshmallow import Marshmallow
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import event
from sqlalchemy.engine import Engine

# BASE_DIR should be the working directory of the app, e.g. 'flask/'
BASE_DIR = dirname(dirname(abspath(__file__)))  # not used yet

app = Flask(__name__)
app.config.from_object("config.Config")
db = SQLAlchemy(app)

@event.listens_for(Engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    """Enable WAL and a 15s busy timeout on every SQLite connection."""
    if isinstance(dbapi_connection, sqlite3.Connection):
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute("PRAGMA busy_timeout=15000")
        cursor.close()
        # Ensure WAL/SHM files are world-writable across container/host user boundaries
        try:
            db_path = app.config.get("SQLALCHEMY_DATABASE_URI", "").replace("sqlite:///", "")
            if db_path and os.path.exists(db_path):
                for suffix in ["", "-wal", "-shm"]:
                    target = db_path + suffix
                    if os.path.exists(target):
                        os.chmod(target, 0o666)
        except Exception:
            pass

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
