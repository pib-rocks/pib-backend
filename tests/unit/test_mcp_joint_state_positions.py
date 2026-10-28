"""PR-2112: MCP list_motors uses atomic /joint_states when fresh, marks fallback."""

from __future__ import annotations

import math
import time

import pytest

from pib_mcp_server.backend import PibBackend
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


def test_stale_joint_states_falls_back_and_marks_source(monkeypatch):
    motors = full_motor_fixture()
    backend = PibBackend()
    backend.joint_state_max_age_sec = 0.1
    bridge = FakeRosbridge(_joint_state_message(motors), received_age_sec=0.5)
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    service_calls: list[str] = []

    def fake_service(service, service_type, arguments):
        service_calls.append(arguments["joint_name"])
        index = service_calls.index(arguments["joint_name"])
        return {"successful": True, "position": 100 * (index + 1)}

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert len(service_calls) == MOTOR_COUNT
    assert all(m.get("positionSource") == "get_joint_position" for m in enriched)
    assert all("positionAgeSec" not in m for m in enriched)


def test_silent_joint_states_falls_back_without_exception(monkeypatch):
    motors = full_motor_fixture()[:3]
    backend = PibBackend()
    bridge = FakeRosbridge(None)
    monkeypatch.setattr(backend, "get_motors", lambda: list(motors))
    monkeypatch.setattr(backend, "_rosbridge_client", lambda: bridge)

    def fake_service(service, service_type, arguments):
        return {"successful": True, "position": 42}

    monkeypatch.setattr(backend, "_ros_service", fake_service)

    enriched = backend._motors_with_live_positions()
    assert len(enriched) == 3
    assert all(m.get("positionSource") == "get_joint_position" for m in enriched)
