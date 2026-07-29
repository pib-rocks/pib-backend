import pytest
import json
from unittest.mock import patch, MagicMock
from app.app import app


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
        response = client.post("/docker/containers/pib-backend/start")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "started" in data["message"]


def test_start_container_not_found(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(404, b"No such container"),
    ):
        response = client.post("/docker/containers/nonexistent/start")
        assert response.status_code == 404
        data = response.get_json()
        assert data["status"] == "error"


def test_stop_container_success(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(204, b""),
    ):
        response = client.post("/docker/containers/pib-backend/stop")
        assert response.status_code == 200
        data = response.get_json()
        assert data["status"] == "success"
        assert "stopped" in data["message"]


def test_restart_container_success(client):
    with patch(
        "service.docker_admin_service._docker_request",
        return_value=(204, b""),
    ):
        response = client.post("/docker/containers/pib-backend/restart")
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
        response = client.get("/docker/containers/pib-backend/logs?tail=100")
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
                response = client.post("/docker/containers/pib-backend/clear-logs")
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
