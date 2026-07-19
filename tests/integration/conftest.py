"""Pytest fixtures for in-process Flask API contract tests."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Generator

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
BLOCKLY_CLIENT_DIR = REPO_ROOT / "pib_blockly" / "pib_blockly_client"
API_CLIENT_DIR = REPO_ROOT / "pib_api" / "client"

for path in (str(FLASK_DIR), str(BLOCKLY_CLIENT_DIR), str(API_CLIENT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

import run  # noqa: E402,F401
import app as _flask_blueprints  # noqa: E402,F401

from app.app import app as flask_app  # noqa: E402
from app.app import db  # noqa: E402
from click.testing import CliRunner  # noqa: E402
from commands import seed_db  # noqa: E402


@pytest.fixture()
def app(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    db_file = tmp_path / "test.db"
    programs_dir = tmp_path / "programs"
    programs_dir.mkdir()
    host_ip_file = tmp_path / "host_ip.txt"
    host_ip_file.write_text("192.168.1.100", encoding="utf-8")

    monkeypatch.setenv("SQLALCHEMY_DATABASE_URI", f"sqlite:///{db_file}")
    monkeypatch.setenv("PYTHON_CODE_DIR", str(programs_dir))
    monkeypatch.setenv("HOST_IP_FILE", str(host_ip_file))

    flask_app.config.update(
        TESTING=True,
        SQLALCHEMY_DATABASE_URI=f"sqlite:///{db_file}",
        PYTHON_CODE_DIR=str(programs_dir),
        HOST_IP_FILE=str(host_ip_file),
    )

    with flask_app.app_context():
        db.drop_all()
        db.create_all()
        result = CliRunner().invoke(seed_db, [])
        if result.exception:
            raise result.exception
        db.session.commit()

    yield flask_app

    with flask_app.app_context():
        db.session.remove()


@pytest.fixture()
def client(app) -> Generator:
    with app.test_client() as test_client:
        yield test_client


@pytest.fixture()
def programs_dir(app) -> Path:
    return Path(app.config["PYTHON_CODE_DIR"])
