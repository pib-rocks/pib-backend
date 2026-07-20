"""Shared paths and compose parsing for infrastructure tests."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"


@pytest.fixture(scope="session")
def compose_dict() -> dict:
    with COMPOSE_FILE.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def docker_available() -> bool:
    try:
        import docker

        docker.from_env().ping()
        return True
    except Exception:
        return False


def pytest_configure(config):
    config.addinivalue_line("markers", "docker: requires Docker daemon")
    config.addinivalue_line("markers", "slow: long-running container tests")
