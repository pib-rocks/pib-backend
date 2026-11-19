import atexit
import logging
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

SQLITE_BUSY_TIMEOUT_MS = 15000
SQLITE_WAL_AUTOCHECKPOINT_PAGES = 200
SQLITE_JOURNAL_SIZE_LIMIT_BYTES = 1024 * 1024  # 1 MiB

_LOGGER = logging.getLogger(__name__)

app = Flask(__name__)
app.config.from_object("config.Config")
db = SQLAlchemy(app)


def _sqlite_db_path(flask_app: Flask = app) -> str:
    uri = str(flask_app.config.get("SQLALCHEMY_DATABASE_URI", "") or "")
    if not uri.startswith("sqlite:///"):
        return ""
    path = uri.replace("sqlite:///", "", 1)
    if not path or path == ":memory:":
        return ""
    return path


def _chmod_sqlite_files(db_path: str) -> None:
    """Ensure DB/WAL/SHM files stay world-writable across container/host user boundaries."""
    if not db_path or not os.path.exists(db_path):
        return
    try:
        for suffix in ("", "-wal", "-shm"):
            target = db_path + suffix
            if os.path.exists(target):
                os.chmod(target, 0o666)
    except Exception:
        pass


def checkpoint_sqlite_wal(
    flask_app: Flask = app, mode: str = "TRUNCATE"
) -> tuple[int, int, int] | None:
    """Checkpoint and bound the SQLite WAL file (best-effort, safe for non-SQLite URIs)."""
    allowed_modes = {"PASSIVE", "FULL", "RESTART", "TRUNCATE"}
    normalized = str(mode).upper()
    if normalized not in allowed_modes:
        raise ValueError(f"Unsupported SQLite WAL checkpoint mode: {mode!r}")
    db_path = _sqlite_db_path(flask_app)
    if not db_path or not os.path.exists(db_path):
        return None
    conn: sqlite3.Connection | None = None
    try:
        conn = sqlite3.connect(db_path, timeout=SQLITE_BUSY_TIMEOUT_MS / 1000.0)
        cursor = conn.cursor()
        cursor.execute(f"PRAGMA busy_timeout={SQLITE_BUSY_TIMEOUT_MS}")
        row = cursor.execute(f"PRAGMA wal_checkpoint({normalized})").fetchone()
        cursor.close()
        _chmod_sqlite_files(db_path)
        if row is not None and len(row) == 3:
            return (int(row[0]), int(row[1]), int(row[2]))
        return None
    except Exception as exc:  # noqa: BLE001
        _LOGGER.debug("SQLite WAL checkpoint (%s) skipped: %s", normalized, exc)
        return None
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass


@event.listens_for(Engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    """Enable WAL, bounded WAL auto-checkpointing, and a 15s busy timeout on SQLite."""
    if isinstance(dbapi_connection, sqlite3.Connection):
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute(f"PRAGMA busy_timeout={SQLITE_BUSY_TIMEOUT_MS}")
        cursor.execute(f"PRAGMA wal_autocheckpoint={SQLITE_WAL_AUTOCHECKPOINT_PAGES}")
        cursor.execute(f"PRAGMA journal_size_limit={SQLITE_JOURNAL_SIZE_LIMIT_BYTES}")
        cursor.close()
        _chmod_sqlite_files(_sqlite_db_path(app))


ma = Marshmallow(app)
migrate = Migrate(app, db)
CORS(app)


if not os.path.exists(app.config.get("PYTHON_CODE_DIR")):
    os.makedirs(app.config.get("PYTHON_CODE_DIR"))

# Imported at the bottom to prevent a circular import error
from commands import reconcile_system_properties, seed_db, seed_hardware

app.cli.add_command(seed_db)
app.cli.add_command(seed_hardware)
app.cli.add_command(reconcile_system_properties)


@app.after_request
def handle_session_after_request(response: Response) -> Response:
    if 200 <= response.status_code < 300:
        db.session.commit()
    else:
        db.session.rollback()
    return response


@app.teardown_appcontext
def cleanup_session_teardown(exception=None) -> None:
    """Release SQLAlchemy sessions on every teardown so failed requests never pin SQLite WAL."""
    if exception is not None:
        try:
            db.session.rollback()
        except Exception:
            pass
    try:
        db.session.remove()
    except Exception:
        pass


checkpoint_sqlite_wal(app, mode="TRUNCATE")
atexit.register(checkpoint_sqlite_wal, app, "TRUNCATE")
