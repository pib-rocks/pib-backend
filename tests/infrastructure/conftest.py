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


def docker_unavailable_reason() -> str:
    """Why the Docker tests cannot run here - empty when Docker is usable."""
    try:
        import docker  # noqa: F401
    except ImportError as exc:
        return f"the docker package is not installed in this interpreter ({exc})"
    try:
        docker.from_env().ping()
    except Exception as exc:  # noqa: BLE001
        return f"the Docker daemon is not reachable ({type(exc).__name__}: {exc})"
    return ""


def docker_available() -> bool:
    return not docker_unavailable_reason()


def pytest_configure(config):
    config.addinivalue_line("markers", "docker: requires Docker daemon")
    config.addinivalue_line("markers", "slow: long-running container tests")
