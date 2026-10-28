"""Flask-app APP_VERSION compose fallback must not claim a release (PR-2111)."""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE = REPO_ROOT / "docker-compose.yaml"

APP_VERSION_DEFAULT = re.compile(
    r"^\s*APP_VERSION:\s*\$\{APP_VERSION:-([^}]+)\}\s*$",
    re.MULTILINE,
)
RELEASE_LITERAL_FALLBACK = re.compile(
    r"^\s*APP_VERSION:\s*\$\{APP_VERSION:-v\d+\.\d+\.\d+\}\s*$"
)


def test_flask_app_app_version_default_is_dev() -> None:
    text = COMPOSE.read_text(encoding="utf-8")
    match = APP_VERSION_DEFAULT.search(text)
    assert match is not None, "missing flask-app APP_VERSION build arg line"
    assert match.group(1) == "dev"
    assert RELEASE_LITERAL_FALLBACK.search(text) is None
