"""Unit tests for Docker admin container name allowlist (PR-2119)."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
if str(FLASK_DIR) not in sys.path:
    sys.path.insert(0, str(FLASK_DIR))

from service import docker_admin_service  # noqa: E402


@pytest.mark.parametrize(
    "name",
    [
        "multirepo-flask-app-1",
        "cerebra-angular-app-1",
        "pib-backend-ros-motors-1",
        "ros-camera",
        "flask-app",
        "pib_motors_dev",
    ],
)
def test_is_allowed_container_name_accepts_managed_names(name):
    assert docker_admin_service.is_allowed_container_name(name) is True


@pytest.mark.parametrize(
    "name",
    [
        "evil_container",
        "../etc/passwd",
        "portainer",
        "root-shell",
        "multirepo-flask-app-1/exec",
        "",
        " bad-name",
    ],
)
def test_is_allowed_container_name_rejects_unmanaged_names(name):
    assert docker_admin_service.is_allowed_container_name(name) is False


@pytest.mark.parametrize(
    "name,method",
    [
        ("evil_container", "start_container"),
        ("portainer", "stop_container"),
        ("root-shell", "restart_container"),
        ("../etc/passwd", "get_container_logs"),
        ("multirepo-flask-app-1/exec", "clear_container_logs"),
    ],
)
def test_container_actions_reject_disallowed_names_without_docker(name, method):
    with patch.object(docker_admin_service, "_docker_request") as docker_request:
        func = getattr(docker_admin_service, method)
        if method == "get_container_logs":
            code, body = func(name)
        else:
            code, body = func(name)

    docker_request.assert_not_called()
    assert code == 403
    assert body["status"] == "error"
    assert "is not managed by pib" in body["message"]
    assert name in body["message"]


@pytest.mark.parametrize(
    "name,method,expected_path",
    [
        (
            "multirepo-flask-app-1",
            "start_container",
            "/containers/multirepo-flask-app-1/start",
        ),
        ("ros-camera", "stop_container", "/containers/ros-camera/stop?t=5"),
        (
            "pib-backend-ros-motors-1",
            "restart_container",
            "/containers/pib-backend-ros-motors-1/restart?t=5",
        ),
        (
            "cerebra-angular-app-1",
            "get_container_logs",
            "/containers/cerebra-angular-app-1/logs?stdout=1&stderr=1&tail=500",
        ),
    ],
)
def test_container_actions_call_docker_for_allowed_names(name, method, expected_path):
    docker_status = 200 if method == "get_container_logs" else 204
    with patch.object(
        docker_admin_service, "_docker_request", return_value=(docker_status, b"")
    ) as docker_request:
        func = getattr(docker_admin_service, method)
        if method == "get_container_logs":
            code, body = func(name, tail=500)
            assert code == 200
            assert body["status"] == "success"
        else:
            code, _body = func(name)
            assert code == 200

    docker_request.assert_called_once()
    call_path = docker_request.call_args[0][1]
    assert call_path == expected_path
