"""Live Docker deployment tests — docs/test-basis/infrastructure_and_deployment.md"""

from __future__ import annotations

import os
import socket
import subprocess
import time
from pathlib import Path
from typing import Any

import pytest
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"
COMPOSE_PORTS_FILE = Path(__file__).resolve().parent / "docker-compose.test-ports.yaml"


def docker_available() -> bool:
    try:
        import docker

        docker.from_env().ping()
        return True
    except Exception:
        return False


def _compose_cmd(*args: str) -> list[str]:
    return [
        "docker",
        "compose",
        "-f",
        str(COMPOSE_FILE),
        "-f",
        str(COMPOSE_PORTS_FILE),
        *args,
    ]


def _container_ip(client: Any, project: str, service: str) -> str:
    network_name = f"{project}_pib-network"
    matches = [
        c
        for c in client.containers.list(all=True)
        if project in c.name and service in c.name and c.status == "running"
    ]
    assert matches, f"running container for {service} not found in project {project}"
    container = matches[0]
    networks = container.attrs["NetworkSettings"]["Networks"]
    assert network_name in networks, f"{container.name} not attached to {network_name}"
    ip = networks[network_name]["IPAddress"]
    assert ip, f"no IP for {container.name} on {network_name}"
    return ip


def _wait_for_flask(project: str, timeout_sec: float = 180) -> str:
    import docker

    client = docker.from_env()
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        try:
            ip = _container_ip(client, project, "flask-app")
            response = requests.get(f"http://{ip}:5000/motor", timeout=3)
            if response.status_code == 200:
                return ip
        except (AssertionError, requests.RequestException):
            pass
        time.sleep(3)
    raise TimeoutError(f"flask-app not ready within {timeout_sec}s for project {project}")


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
        _compose_cmd("down", "-v"),
        cwd=REPO_ROOT,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    subprocess.run(
        _compose_cmd(
            "up",
            "-d",
            "--build",
            "pib-blockly-server",
            "flask-app",
            "rosbridge-ws",
        ),
        cwd=REPO_ROOT,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )

    try:
        flask_ip = _wait_for_flask(compose_project_name)
    except TimeoutError:
        subprocess.run(_compose_cmd("logs", "flask-app"), cwd=REPO_ROOT, env=env)
        subprocess.run(_compose_cmd("down", "-v"), cwd=REPO_ROOT, env=env)
        pytest.fail("flask-app did not become ready within 180 seconds")

    import docker

    client = docker.from_env()
    yield {
        "project": compose_project_name,
        "flask_ip": flask_ip,
        "blockly_ip": _container_ip(client, compose_project_name, "pib-blockly-server"),
        "rosbridge_ip": _container_ip(client, compose_project_name, "rosbridge-ws"),
    }

    subprocess.run(_compose_cmd("down", "-v"), cwd=REPO_ROOT, env=env, check=False)


@pytest.mark.slow
class TestDockerDeployment:
    def test_flask_motor_endpoint_responds(self, compose_stack):
        ip = compose_stack["flask_ip"]
        response = requests.get(f"http://{ip}:5000/motor", timeout=10)
        assert response.status_code == 200
        assert "motors" in response.json()

    def test_blockly_server_port_open(self, compose_stack):
        ip = compose_stack["blockly_ip"]
        sock = socket.create_connection((ip, 2442), timeout=5)
        sock.close()

    def test_rosbridge_port_open(self, compose_stack):
        ip = compose_stack["rosbridge_ip"]
        sock = socket.create_connection((ip, 9090), timeout=5)
        sock.close()

    def test_containers_on_pib_network(self, compose_stack):
        import docker

        client = docker.from_env()
        project = compose_stack["project"]
        network_name = f"{project}_pib-network"
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
        project = compose_stack["project"]
        flask_containers = [
            c
            for c in client.containers.list(all=True)
            if project in c.name and "flask-app" in c.name
        ]
        assert flask_containers, "flask-app container not found"
        container = flask_containers[0]
        subprocess.run(["docker", "restart", container.id], check=True)
        try:
            ip = _wait_for_flask(project, timeout_sec=120)
        except TimeoutError:
            container.reload()
            logs = container.logs(tail=40).decode("utf-8", errors="replace")
            pytest.fail(
                "flask-app did not recover within 120s after restart "
                f"(status={container.status}). Recent logs:\n{logs}"
            )
        response = requests.get(f"http://{ip}:5000/motor", timeout=15)
        assert response.status_code == 200


class TestComposeConfigValid:
    def test_docker_compose_config_succeeds(self):
        if not docker_available():
            pytest.skip("Docker daemon not available")
        result = subprocess.run(
            _compose_cmd("config"),
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
