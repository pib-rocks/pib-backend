import pytest
import json
from unittest.mock import patch, MagicMock
from app.app import app

# docker_admin_service.is_allowed_container_name() answers 403 for anything outside
# ALLOWED_CONTAINER_PREFIXES / ALLOWED_CONTAINER_NAMES, so these tests must use a
# name the API actually manages - 'pib-backend' alone is not one of them (PR-2165).
MANAGED_CONTAINER = "pib-backend-ros-motors-1"
MISSING_MANAGED_CONTAINER = "pib-backend-no-such-container-1"
UNMANAGED_CONTAINER = "nginx"


@pytest.fixture
def client():
    app.config["TESTING"] = True
    with app.test_client() as client:
        yield client


def test_get_containers(client):
    mock_containers_raw = [
        {
            "Id": "1234567890ab1234567890",
            "Names": ["/pib-backend"],
            "Image": "pib-backend:latest",
            "State": "running",
            "Status": "Up 2 hours (healthy)",
            "Created": 1700000000,
        },
        {
            "Id": "abcdef123456abcdef123456",
            "Names": ["/rosbridge"],
            "Image": "rosbridge:latest",
            "State": "running",
            "Status": "Up 2 hours",
            "Created": 1700000000,
        },
    ]

    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(200, json.dumps(mock_containers_raw).encode("utf-8")),
    ):
        response = client.get("/docker/containers")
        assert response.status_code == 200
        data = response.get_json()
        assert len(data) == 2
        assert data[0]["name"] == "pib-backend"
        assert data[0]["status"] == "running"
        assert data[0]["health"] == "healthy"
        assert data[1]["name"] == "rosbridge"


def test_start_container_success(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(204, b""),
    ):
        response = client.post(f"/docker/containers/{MANAGED_CONTAINER}/start")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "started" in data["message"]


def test_start_container_not_found(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(404, b"No such container"),
    ):
        response = client.post(f"/docker/containers/{MISSING_MANAGED_CONTAINER}/start")
        assert response.status_code == 404
        data = response.get_json()
        assert data["status"] == "error"


def test_unmanaged_container_is_refused_with_403(client):
    """The allowlist guard answers 403 before Docker is asked anything."""
    with patch(
        "service.docker_admin_service._docker_request",
        side_effect=AssertionError(
            "an unmanaged name must not reach the Docker socket"
        ),
    ):
        for action in ("start", "stop", "restart"):
            response = client.post(f"/docker/containers/{UNMANAGED_CONTAINER}/{action}")
            assert response.status_code == 403, action
            data = response.get_json()
            assert data["status"] == "error"
            assert "not managed by pib" in data["message"]


def test_stop_container_success(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(204, b""),
    ):
        response = client.post(f"/docker/containers/{MANAGED_CONTAINER}/stop")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "stopped" in data["message"]


def test_restart_container_success(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(204, b""),
    ):
        response = client.post(f"/docker/containers/{MANAGED_CONTAINER}/restart")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "restarted" in data["message"]


def test_get_container_logs(client):
    raw_logs = b"\x01\x00\x00\x00\x00\x00\x00\x0cHello World\n"
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(200, raw_logs),
    ):
        response = client.get(f"/docker/containers/{MANAGED_CONTAINER}/logs?tail=100")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "Hello World" in data["logs"]


def test_clear_container_logs(client):
    mock_info = {"LogPath": "/tmp/test-container-log.log"}
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(200, json.dumps(mock_info).encode("utf-8")),
    ):
        with patch("os.path.exists", return_value=True):
            with patch("builtins.open", MagicMock()):
                response = client.post(
                    f"/docker/containers/{MANAGED_CONTAINER}/clear-logs"
                )
                assert response.status_code == 200
                data = response.get_json()
                assert data["status"] == "success"
                assert "cleared" in data["message"]


def test_purge_docker(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(200, json.dumps({"ContainersDeleted": []}).encode("utf-8")),
    ):
        response = client.post("/docker/admin/purge")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "completed" in data["message"]
