"""Every interface package the MCP tools call over rosbridge must be built into that image.

rosbridge builds the request for a ``call_service`` from the service's ``.srv`` type, so an
interface package missing from the image makes the call fail *before* it reaches the node:

    Unable to import button_service.srv from package button_service. Caused by:
    No module named 'button_service'

Measured on the robot: ``/tf_button/set_color`` answered ``success=True, message='OK'`` when called
inside the container that runs the node, while every call through rosbridge failed with the import
error - so a healthy service looked broken to the chat tools.

The required packages are derived from the MCP backend itself, so a new tool that calls a service
from a new package is covered without editing this test.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BACKEND = REPO_ROOT / "pib_mcp_server" / "backend.py"
ROSBRIOGE_DOCKERFILE = REPO_ROOT / "ros_packages" / "rosbridge" / "Dockerfile"

# _ros_service("/service", "package/srv/Type", {...})
SERVICE_CALL = re.compile(
    r'_ros_service\(\s*\n?\s*"(?P<service>[^"]+)"\s*,\s*\n?\s*"(?P<type>[^"]+)"',
    re.MULTILINE,
)
WORKSPACE_COPY = re.compile(r"^COPY \./(?P<package>\S+) ros2_ws/\S+$", re.MULTILINE)
COLCON_BUILD = re.compile(r"colcon build")


def rosbridge_service_packages() -> dict[str, set[str]]:
    """Interface package -> the services the MCP backend calls with a type from it."""
    usage: dict[str, set[str]] = {}
    for match in SERVICE_CALL.finditer(BACKEND.read_text(encoding="utf-8")):
        package = match.group("type").split("/", 1)[0]
        usage.setdefault(package, set()).add(match.group("service"))
    return usage


def rosbridge_built_packages() -> set[str]:
    """Interface packages the rosbridge Dockerfile copies into its colcon workspace."""
    return set(WORKSPACE_COPY.findall(ROSBRIOGE_DOCKERFILE.read_text(encoding="utf-8")))


def test_the_parser_finds_the_service_calls_it_guards():
    """A vacuous parse would make every assertion below meaningless."""
    usage = rosbridge_service_packages()
    assert (
        usage
    ), "no _ros_service(service, type) call found in pib_mcp_server/backend.py"
    assert "datatypes" in usage, f"expected the datatypes calls, found {sorted(usage)}"


def test_every_interface_package_used_over_rosbridge_is_built_into_the_image():
    usage = rosbridge_service_packages()
    built = rosbridge_built_packages()
    missing = {
        package: sorted(services)
        for package, services in usage.items()
        if package not in built
    }
    assert not missing, (
        "these interface packages are called over rosbridge but not built in "
        f"{ROSBRIOGE_DOCKERFILE.relative_to(REPO_ROOT)}: {missing}; every call of their services "
        "fails with 'Unable to import <pkg>.srv from package <pkg>' before it reaches the node"
    )


def test_the_dockerfile_actually_builds_its_workspace():
    """Copying a package without building it leaves the srv type unavailable."""
    text = ROSBRIOGE_DOCKERFILE.read_text(encoding="utf-8")
    assert (
        rosbridge_built_packages()
    ), "the rosbridge Dockerfile copies no package into its workspace"
    assert COLCON_BUILD.search(text), "the rosbridge Dockerfile never runs colcon build"


def test_the_button_service_interfaces_reach_rosbridge():
    """The measured regression: /tf_button/set_color must be callable over rosbridge."""
    assert "button_service" in rosbridge_built_packages(), (
        "button_service is missing from the rosbridge image, so /tf_button/set_color "
        "(SetButtonColor) cannot be called over the websocket"
    )
