"""Live Docker deployment tests — docs/test-basis/infrastructure_and_deployment.md"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

import pytest
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"


def docker_available() -> bool:
    try:
        import docker

        docker.from_env().ping()
        return True
    except Exception:
        return False


pytestmark = pytest.mark.docker


@pytest.fixture(scope="module")
def compose_project_name() -> str:
    return f"pibtest_{os.getpid()}"


@pytest.fixture(scope="module")
def compose_stack(compose_project_name):
    if not docker_available():
        pytest.skip("Docker daemon not available")
    if os.getenv("RUN_DOCKER_TESTS", "0") != "1":
        pytest.skip("Set RUN_DOCKER_TESTS=1 to run live Docker deployment tests")

    env = os.environ.copy()
    env["COMPOSE_PROJECT_NAME"] = compose_project_name

    subprocess.run(
        [
            "docker",
            "compose",
            "-f",
            str(COMPOSE_FILE),
            "up",
            "-d",
            "--build",
            "pib-blockly-server",
            "flask-app",
            "rosbridge-ws",
        ],
        cwd=REPO_ROOT,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )

    deadline = time.time() + 180
    flask_ready = False
    while time.time() < deadline:
        try:
            response = requests.get("http://localhost:5000/motor", timeout=3)
            if response.status_code == 200:
                flask_ready = True
                break
        except requests.RequestException:
            pass
        time.sleep(3)

    if not flask_ready:
        subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "logs", "flask-app"],
            cwd=REPO_ROOT,
            env=env,
        )
        subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "down", "-v"],
            cwd=REPO_ROOT,
            env=env,
        )
        pytest.fail("flask-app did not become ready within 180 seconds")

    yield compose_project_name

    subprocess.run(
        ["docker", "compose", "-f", str(COMPOSE_FILE), "down", "-v"],
        cwd=REPO_ROOT,
        env=env,
        check=False,
    )


@pytest.mark.slow
class TestDockerDeployment:
    def test_flask_motor_endpoint_responds(self, compose_stack):
        response = requests.get("http://localhost:5000/motor", timeout=10)
        assert response.status_code == 200
        assert "motors" in response.json()

    def test_blockly_server_port_open(self, compose_stack):
        import socket

        sock = socket.create_connection(("localhost", 2442), timeout=5)
        sock.close()

    def test_rosbridge_port_open(self, compose_stack):
        import socket

        sock = socket.create_connection(("localhost", 9090), timeout=5)
        sock.close()

    def test_containers_on_pib_network(self, compose_stack):
        import docker

        client = docker.from_env()
        network_name = f"{compose_stack}_pib-network"
        network = client.networks.get(network_name)
        containers = client.containers.list(
            filters={"network": network.id, "status": "running"}
        )
        names = {c.name for c in containers}
        assert any("flask-app" in n for n in names)
        assert any("pib-blockly-server" in n for n in names)
        assert any("rosbridge-ws" in n for n in names)

    @pytest.mark.slow
    def test_flask_container_restarts_after_kill(self, compose_stack):
        import docker

        client = docker.from_env()
        flask_containers = [
            c
            for c in client.containers.list(all=True)
            if compose_stack in c.name and "flask-app" in c.name
        ]
        assert flask_containers, "flask-app container not found"
        container = flask_containers[0]
        container.kill()
        time.sleep(8)
        container.reload()
        assert container.status == "running"
        response = requests.get("http://localhost:5000/motor", timeout=15)
        assert response.status_code == 200


class TestComposeConfigValid:
    def test_docker_compose_config_succeeds(self):
        if not docker_available():
            pytest.skip("Docker daemon not available")
        result = subprocess.run(
            ["docker", "compose", "-f", str(COMPOSE_FILE), "config"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
