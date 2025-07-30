import os
import pytest
from pib_api.flask.app.app import app


class TestConfig:
    TESTING = True
    HOST_IP_FILE = "/some/path/host_ip.txt"


@pytest.fixture
def client(tmp_path):
    app.config.from_object(TestConfig)
    app.config["HOST_IP_FILE"] = str(tmp_path / "host_ip.txt")

    with app.test_client() as client:
        yield client


def test_host_ip_with_file(client):
    ip_file = client.application.config["HOST_IP_FILE"]
    with open(ip_file, "w") as f:
        f.write("123.456.789.0")

    response = client.get("/host-ip")

    assert response.status_code == 200
    assert response.json == {"host_ip": "123.456.789.0"}


def test_host_ip_no_file(client, tmp_path):
    ip_file = client.application.config["HOST_IP_FILE"]

    if os.path.exists(ip_file):
        os.unlink(ip_file)

    response = client.get("/host-ip")

    assert response.status_code == 200
    assert response.json == {"host_ip": ""}


def test_host_ip_empty_file(client):
    ip_file = client.application.config["HOST_IP_FILE"]
    with open(ip_file, "w") as f:
        f.write("")

    response = client.get("/host-ip")

    assert response.status_code == 200
    assert response.json == {"host_ip": ""}
