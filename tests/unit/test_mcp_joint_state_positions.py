"""PR-2112: MCP list_motors uses atomic /joint_states when fresh, marks fallback."""

from __future__ import annotations

import math
import time

import pytest

from pib_mcp_server.backend import BackendError, PibBackend
from test_list_motors_compact import MOTOR_COUNT, full_motor_fixture


def _joint_state_message(motors: list[dict], centideg_per_motor: int = 120) -> dict:
    names = []
    positions = []
    for index, motor in enumerate(motors):
        name = motor["name"]
        names.append(name)
        rad = math.radians(centideg_per_motor * (index + 1) / 100.0)
        positions.append(rad)
    now = time.time()
    return {
        "header": {
            "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
        },
        "name": names,
        "position": positions,
    }


class FakeRosbridge:
    def __init__(self, msg: dict | None, *, received_age_sec: float = 0.0):
        self._msg = msg
        self._received_age_sec = received_age_sec
        self.subscribe_topics: list[tuple[str, str]] = []

    def subscribe_topic(self, topic: str, message_type: str) -> None:
        self.subscribe_topics.append((topic, message_type))

    def latest_topic_message(self, topic: str):
        if self._msg is None:
            return None
        return {
            "msg": self._msg,
            "received_monotonic": time.monotonic() - self._received_age_sec,
        }


def test_joint_states_path_avoids_per_joint_service_calls(monkeypatch):
    motors = full_motor_fixture()
    backend = PibBackend()
    backend.joint_state_max_age_sec = 1.0
    bridge = FakeRosbridge(_joint_state_message(motors))
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    service_calls: list[str] = []

    def fake_service(service, service_type, arguments):
        service_calls.append(arguments["joint_name"])
        return {"successful": True, "position": 0}

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert service_calls == []
    assert bridge.subscribe_topics == [("/joint_states", "sensor_msgs/JointState")]
    for index, motor in enumerate(enriched):
        assert motor.get("positionSource") == "joint_states"
        assert motor.get("positionAgeSec") is not None
        assert motor["currentPosition"] == pytest.approx(1.2 * (index + 1))


def test_stale_joint_states_uses_batch_service_before_per_joint(monkeypatch):
    motors = full_motor_fixture()
    backend = PibBackend()
    backend.joint_state_max_age_sec = 0.1
    bridge = FakeRosbridge(_joint_state_message(motors), received_age_sec=0.5)
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    service_calls: list[tuple[str, dict]] = []

    def fake_service(service, service_type, arguments):
        service_calls.append((service, dict(arguments)))
        if service == "/get_all_joint_positions":
            now = time.time()
            results = []
            positions = []
            for index, motor in enumerate(motors):
                name = motor["name"]
                results.append(
                    {
                        "motor_name": name,
                        "successful": True,
                        "detail": "",
                        "error_code": "",
                    }
                )
                positions.append(100 * (index + 1))
            return {
                "successful": True,
                "header": {
                    "stamp": {
                        "sec": int(now),
                        "nanosec": int((now % 1) * 1e9),
                    }
                },
                "results": results,
                "positions": positions,
                "message": "",
            }
        return {"successful": True, "position": 0}

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert len(service_calls) == 1
    assert service_calls[0][0] == "/get_all_joint_positions"
    assert all(m.get("positionSource") == "get_all_joint_positions" for m in enriched)
    assert all(m.get("positionAgeSec") is not None for m in enriched)


def test_silent_joint_states_falls_back_without_exception(monkeypatch):
    motors = full_motor_fixture()[:3]
    backend = PibBackend()
    bridge = FakeRosbridge(None)
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    def fake_service(service, service_type, arguments):
        if service == "/get_all_joint_positions":
            raise BackendError(
                "ros_service_failed",
                "Service /get_all_joint_positions does not exist",
            )
        return {"successful": True, "position": 42}

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert len(enriched) == 3
    assert all(m.get("positionSource") == "get_joint_position" for m in enriched)
    assert all(
        m.get("positionFallbackReason")
        == "get_all_joint_positions service unavailable (older motors image)"
        for m in enriched
    )


def test_batch_service_reports_per_joint_errors_without_failing_others(monkeypatch):
    motors = full_motor_fixture()[:2]
    backend = PibBackend()
    bridge = FakeRosbridge(None)
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    def fake_service(service, service_type, arguments):
        assert service == "/get_all_joint_positions"
        now = time.time()
        return {
            "successful": True,
            "header": {
                "stamp": {"sec": int(now), "nanosec": 0},
            },
            "results": [
                {
                    "motor_name": motors[0]["name"],
                    "successful": True,
                    "error_code": "",
                    "detail": "",
                },
                {
                    "motor_name": motors[1]["name"],
                    "successful": False,
                    "error_code": "position_unavailable",
                    "detail": "no measurement in bus sweep",
                },
            ],
            "positions": [500, 0],
            "message": "",
        }

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert enriched[0]["currentPosition"] == pytest.approx(5.0)
    assert enriched[0].get("positionSource") == "get_all_joint_positions"
    assert enriched[1]["currentPosition"] is None
    assert enriched[1]["positionError"] == "no measurement in bus sweep"


def _backend_with_motors(monkeypatch, motors):
    from pib_mcp_server import backend as backend_module

    monkeypatch.setattr(backend_module.PibBackend, "get_motors", lambda self: motors)
    monkeypatch.setattr(
        backend_module.PibBackend,
        "_try_apply_joint_state_positions",
        lambda self, named: (False, None),
    )
    return backend_module.PibBackend(), backend_module


def test_all_unsuccessful_batch_falls_back_to_the_per_joint_reads(monkeypatch):
    """A pib5edu answers the batch service with no usable value at all.

    Measured on the robot: the bricklets have no position feedback, so every entry is
    unsuccessful. Treating that as 'applied' skipped the per-joint path and left every
    position null - the tool reported less than before stage 2.
    """
    motors = [
        {"name": "turn_head_motor", "controller": {"kind": "tinkerforge_bricklet"}}
    ]
    backend, backend_module = _backend_with_motors(monkeypatch, motors)

    monkeypatch.setattr(
        backend_module.PibBackend,
        "_ros_service",
        lambda self, service, service_type, arguments: {
            "successful": False,
            "message": "no sweepable actuator family on this robot",
            "results": [
                {
                    "motor_name": "turn_head_motor",
                    "successful": False,
                    "error_code": "bus_unavailable",
                    "detail": "no sweepable actuator family on this robot",
                }
            ],
            "positions": [0],
        },
    )

    def fake_per_joint(self, motor):
        motor["currentPosition"] = 1234
        motor["positionSource"] = "get_joint_position"

    monkeypatch.setattr(
        backend_module.PibBackend, "_apply_live_joint_position", fake_per_joint
    )

    out = backend.list_motors()

    assert out["motors"][0]["position"] is not None
    assert out["motors"][0]["positionSource"] == "get_joint_position"


def test_usable_batch_values_are_applied_without_the_fallback(monkeypatch):
    motors = [{"name": "elbow_right", "controller": {"kind": "feetech_st_serial"}}]
    backend, backend_module = _backend_with_motors(monkeypatch, motors)

    monkeypatch.setattr(
        backend_module.PibBackend,
        "_ros_service",
        lambda self, service, service_type, arguments: {
            "successful": True,
            "message": "",
            "results": [{"motor_name": "elbow_right", "successful": True}],
            "positions": [2500],
        },
    )
    monkeypatch.setattr(
        backend_module.PibBackend,
        "_apply_live_joint_position",
        lambda self, motor: pytest.fail(
            "the fallback must not run when the batch answered"
        ),
    )

    out = backend.list_motors()

    assert out["motors"][0]["position"] is not None
    assert out["motors"][0]["positionSource"] == "get_all_joint_positions"
