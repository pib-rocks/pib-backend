"""Static contract tests for docker-compose.yaml — docs/test-basis/infrastructure_and_deployment.md"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ENV_FILE = REPO_ROOT / ".env"

DEFAULT_SERVICES = {"pib-blockly-server", "flask-app", "rosbridge-ws"}

PROFILE_ALL_SERVICES = {
    "pib-blockly-server",
    "flask-app",
    "rosbridge-ws",
    "ros-camera",
    "ros-motors",
    "ros-programs",
    "ros-voice-assistant",
    "ros-display",
    "ros-audio-io",
}


class TestComposeArchitecture:
    def test_network_pib_network_exists(self, compose_dict):
        networks = compose_dict.get("networks", {})
        assert "pib-network" in networks
        assert networks["pib-network"].get("driver") == "bridge"

    def test_programs_named_volume(self, compose_dict):
        assert "programs" in compose_dict.get("volumes", {})

    def test_default_services_have_no_profile(self, compose_dict):
        services = compose_dict["services"]
        for name in DEFAULT_SERVICES:
            assert name in services
            assert "profiles" not in services[name]

    def test_profile_gated_services(self, compose_dict):
        services = compose_dict["services"]
        for name in ("ros-motors", "ros-programs", "ros-camera"):
            assert "all" in services[name]["profiles"]

    @pytest.mark.parametrize(
        "service,port",
        [
            ("pib-blockly-server", "2442:2442"),
            ("flask-app", "5000:5000"),
            ("rosbridge-ws", "9090:9090"),
        ],
    )
    def test_exposed_ports(self, compose_dict, service, port):
        published = compose_dict["services"][service]["ports"]
        assert port in published

    def test_all_services_restart_always(self, compose_dict):
        for name, spec in compose_dict["services"].items():
            assert spec.get("restart") == "always", f"{name} missing restart: always"

    def test_flask_depends_on_blockly(self, compose_dict):
        deps = compose_dict["services"]["flask-app"]["depends_on"]
        assert "pib-blockly-server" in deps

    def test_ros_motors_depends_on_flask_and_rosbridge(self, compose_dict):
        deps = compose_dict["services"]["ros-motors"]["depends_on"]
        assert "flask-app" in deps
        assert "rosbridge-ws" in deps

    def test_no_healthcheck_defined(self, compose_dict):
        for name, spec in compose_dict["services"].items():
            assert "healthcheck" not in spec, f"{name} unexpectedly has healthcheck"

    def test_flask_environment_variables(self, compose_dict):
        env = compose_dict["services"]["flask-app"]["environment"]
        assert "SQLALCHEMY_DATABASE_URI=sqlite:////app/pibdata.db" in env
        assert "PIB_BLOCKLY_SERVER_URL=http://pib-blockly-server:2442" in env

    def test_ros_motors_flask_api_url(self, compose_dict):
        env = compose_dict["services"]["ros-motors"]["environment"]
        assert "FLASK_API_BASE_URL=http://flask-app:5000" in env

    def test_ros_programs_shared_program_dir(self, compose_dict):
        spec = compose_dict["services"]["ros-programs"]
        env = spec["environment"]
        assert "PROGRAM_DIR=/ros2_ws/cerebra_programs" in env
        mount_targets = [v.rsplit(":", 1)[-1] for v in spec["volumes"]]
        assert "/ros2_ws/cerebra_programs" in mount_targets

    def test_compose_project_name_in_env_file(self):
        content = ENV_FILE.read_text(encoding="utf-8")
        assert "COMPOSE_PROJECT_NAME=multirepo" in content
