"""Pytest fixtures for Flask service unit tests."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Generator

import pytest

# public_api_client.__init__ requires a tryb URL; unit tests never hit the network.
os.environ.setdefault("TRYB_URL_PREFIX", "http://localhost/test")

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
BLOCKLY_CLIENT_DIR = REPO_ROOT / "pib_blockly" / "pib_blockly_client"
API_CLIENT_DIR = REPO_ROOT / "pib_api" / "client"
PUBLIC_API_CLIENT_DIR = REPO_ROOT / "public_api_client"
HERMES_CONFIG_DIR = REPO_ROOT / "pib_hermes_config"

for path in (
    str(FLASK_DIR),
    str(BLOCKLY_CLIENT_DIR),
    str(API_CLIENT_DIR),
    str(PUBLIC_API_CLIENT_DIR),
    str(HERMES_CONFIG_DIR),
):
    if path not in sys.path:
        sys.path.insert(0, path)

import run  # noqa: E402,F401
import app as _flask_blueprints  # noqa: E402,F401

from app.app import app as flask_app  # noqa: E402
from app.app import db  # noqa: E402
from click.testing import CliRunner  # noqa: E402
from commands import seed_db  # noqa: E402
from model.personality_model import Personality  # noqa: E402
from service import personality_service  # noqa: E402


@pytest.fixture(autouse=True)
def skip_account_live_model_pin(monkeypatch: pytest.MonkeyPatch) -> None:
    """Unlock must not call a provider model list. Pin tests call the function."""
    import service.live_model_service as live_model_service

    monkeypatch.setattr(live_model_service, "pin_unlocked_providers", lambda: None)


@pytest.fixture(autouse=True)
def sandboxed_hermes_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Point every Hermes path at the sandbox before a test can forget to.

    The defaults are the robot's real, shared locations, so a test that provisions
    a profile without overriding them writes into the developer's own
    ``~/.hermes``. Tests may still override either variable.
    """
    home = tmp_path / "hermes-home"
    home.mkdir()
    monkeypatch.setenv("HERMES_HOME", str(home))
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(home / "profiles"))
    # A host marker file must not flip channel tests. 1 keeps Smart available.
    monkeypatch.setenv("PIB_SMART_CHATS", "1")
    # Host Gemini keys must not leak into profile-provisioning assertions.
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)
    # A host key-store settings file must not flip encryption off under the tests.
    monkeypatch.setenv(
        "PIB_KEY_STORE_PATH", str(tmp_path / "secrets" / "provider_key_store.json")
    )
    monkeypatch.delenv("PIB_KEY_STORE_SETTINGS_PATH", raising=False)
    return home


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
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", "pib5edu")
    # A host marker file must not flip channel tests. 1 keeps Smart available.
    monkeypatch.setenv("PIB_SMART_CHATS", "1")
    # Keep SOUL materialization inside the test sandbox instead of the robot's
    # real, container-shared profiles directory.
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "hermes-profiles"))
    # Avoid public_api_client import failure when hermes tests pull it in.
    monkeypatch.setenv("TRYB_URL_PREFIX", "http://localhost/test")

    flask_app.config.update(
        TESTING=True,
        SQLALCHEMY_DATABASE_URI=f"sqlite:///{db_file}",
        PYTHON_CODE_DIR=str(programs_dir),
        HOST_IP_FILE=str(host_ip_file),
        SQLALCHEMY_ENGINE_OPTIONS={"connect_args": {"timeout": 15.0}},
    )

    with flask_app.app_context():
        db.session.remove()
        # Flask-SQLAlchemy builds engines in init_app; recreate so the sandboxed
        # URI and connect timeout from config actually take effect.
        engines = db._app_engines[flask_app]
        options = {
            "url": f"sqlite:///{db_file}",
            **flask_app.config.get("SQLALCHEMY_ENGINE_OPTIONS", {}),
        }
        db._apply_driver_defaults(options, flask_app)
        if None in engines:
            engines[None].dispose()
        engines[None] = db._make_engine(None, options, flask_app)

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
def installed_hermes_bin(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A real executable at PIB_HERMES_BIN, so the preflight passes anywhere.

    Tests that mock out subprocess still have to get past the binary check, and
    the developer machines that happen to have hermes installed must not be what
    makes them pass.
    """
    binary = tmp_path / "hermes"
    binary.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    binary.chmod(0o755)
    monkeypatch.setenv("PIB_HERMES_BIN", str(binary))
    return binary


@pytest.fixture()
def app_ctx(app) -> Generator:
    with app.app_context():
        yield


@pytest.fixture()
def make_personality(app_ctx):
    def _make(**kwargs):
        # The default provider is the one the catalogue maintains, pib.Cloud.
        dto = {
            "name": kwargs.get("name", "Test"),
            "gender": kwargs.get("gender", "Female"),
            "pause_threshold": kwargs.get("pause_threshold", 0.8),
            "message_history": kwargs.get("message_history", 5),
            "description": kwargs.get("description", ""),
        }
        if "assistant_model_id" in kwargs:
            dto["assistant_model_id"] = kwargs["assistant_model_id"]
        personality = personality_service.create_personality(dto)
        db.session.commit()
        return personality

    return _make


@pytest.fixture(autouse=True)
def hermes_turn_has_a_provider_key(monkeypatch: pytest.MonkeyPatch) -> None:
    """Existing Hermes tests do not open the key store.

    A turn now asks the store before it starts. This stand-in keeps those
    tests on their previous path. Tests of the store itself replace it.
    """
    from public_api_client import hermes_agent_client

    if not hasattr(hermes_agent_client, "provider_key_for_turn"):
        return
    monkeypatch.setattr(
        hermes_agent_client,
        "provider_key_for_turn",
        lambda: "fixture-hermes-key",
    )
