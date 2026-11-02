"""Integration tests for the installed pib-sdk (PR-2168).

The SDK is a separate repository with its own suite; pib-backend only needs three
things from it here: the package is installed at or above the version floor this
repo declares, the API surface it calls exists, and a command reaches rosbridge in
the shape the current SDK produces (radians, shared client, release on close).

The earlier version of this file asserted SDK internals of an older generation -
``pib_sdk.control._wait_until_connected``, one ``roslibpy.Ros`` per client that is
terminated on exit, and centidegree joint positions - and failed on every host
where the SDK had moved on.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_DIR = REPO_ROOT / "setup"
if str(SETUP_DIR) not in sys.path:
    sys.path.insert(0, str(SETUP_DIR))

# A host without pib-sdk must skip, not error: the SDK is an external package and
# CI installs only the backend's own requirements.
pib_sdk = pytest.importorskip(
    "pib_sdk", reason="pib-sdk is not installed in this environment"
)

#: Floor taken from tests/integration/requirements.txt ("pib-sdk>=0.9").
REQUIRED_SDK_VERSION = "0.9.0"


def test_installed_pib_sdk_meets_the_declared_version_floor():
    from pib_sdk_verify import verify

    ok, message = verify(pib_sdk.__version__, REQUIRED_SDK_VERSION)
    assert ok, message


def test_pib_sdk_exposes_the_api_pib_backend_uses():
    """Every symbol the robot-side tooling imports must exist on this version."""
    for name in (
        "Write",
        "head",
        "right_arm",
        "left_arm",
        "zero_position",
        "control",
        "ros_client",
    ):
        assert hasattr(pib_sdk, name), f"pib_sdk.{name} is missing"

    from pib_sdk import ros_client

    assert callable(getattr(ros_client, "get_ros", None))
    assert callable(getattr(ros_client, "terminate_all", None))


def _write_client_with_mocked_connection():
    """One Write client whose rosbridge connection and services are fakes."""
    shared_ros = MagicMock()
    shared_ros.is_connected = True
    service = MagicMock()
    service.call.return_value = {"successful": True, "settings_applied": True}
    with (
        patch("pib_sdk.control.get_ros", return_value=shared_ros),
        patch("pib_sdk.control.roslibpy.Service", return_value=service),
        patch("pib_sdk.control.roslibpy.Topic"),
    ):
        client = pib_sdk.Write(host="localhost", port=9090)
    client._joint_trajectory_service = service
    client._motor_settings_service = service
    return client, shared_ros, service


def test_move_sends_radians_through_the_joint_trajectory_service():
    """``move`` takes degrees in [-90, 90] and must publish radians."""
    client, shared_ros, service = _write_client_with_mocked_connection()

    assert client.move("head_pitch", -15.0) is True

    assert service.call.call_count == 1
    request = service.call.call_args[0][0]
    trajectory = request["joint_trajectory"]
    assert trajectory["joint_names"] == ["head_pitch"]
    assert trajectory["points"][0]["positions"] == [pytest.approx(math.radians(-15.0))]


def test_move_rejects_a_degree_value_outside_the_documented_range():
    client, _shared_ros, _service = _write_client_with_mocked_connection()

    with pytest.raises(ValueError):
        client.move("head_pitch", 120.0)


def test_set_sends_motor_settings_for_the_named_motor():
    client, _shared_ros, service = _write_client_with_mocked_connection()

    assert client.set("head_pitch", velocity=6000, acceleration=10000) is True

    request = service.call.call_args[0][0]
    settings = request["motor_settings"]
    assert settings["motor_name"] == "head_pitch"
    assert settings["velocity"] == 6000
    assert settings["acceleration"] == 10000


def test_close_releases_the_shared_client_instead_of_terminating_it():
    """The SDK keeps one process-wide connection; close() only decrements it."""
    client, shared_ros, _service = _write_client_with_mocked_connection()

    with patch("pib_sdk.control.release_ros") as release:
        client.close()

    release.assert_called_once_with(shared_ros)
    assert not shared_ros.terminate.called
