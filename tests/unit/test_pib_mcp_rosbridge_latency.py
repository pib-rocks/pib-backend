"""PR-2100: rosbridge connection reuse, concurrent joint reads, compact list_motors."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from typing import Any

import pytest

from pib_mcp_server.backend import JOINT_STATES_TOPIC, PibBackend, _compact_motor
from pib_mcp_server.tool_budgets import payload_utf8_byte_length
from test_list_motors_compact import MOTOR_COUNT, full_motor_fixture


class _FakeRosbridgeSocket:
    """Minimal rosbridge peer: one recv queue, service replies keyed by request id."""

    instances: list["_FakeRosbridgeSocket"] = []

    def __init__(self) -> None:
        self._outbound: queue.Queue[str] = queue.Queue()
        self._lock = threading.Lock()
        _FakeRosbridgeSocket.instances.append(self)

    def send(self, payload: str) -> None:
        request = json.loads(payload)
        if request.get("op") == "subscribe":
            if request.get("topic") == JOINT_STATES_TOPIC:
                self._enqueue_joint_states_publish()
            return

        service = request.get("service", "")
        joint_names = request.get("args", {}).get("joint_names")
        if service.endswith("get_all_joint_positions") or joint_names is not None:
            delay = self._batch_delay
            if delay:

                def _respond() -> None:
                    time.sleep(delay)
                    self._enqueue_service_response(request)

                threading.Thread(target=_respond, daemon=True).start()
            else:
                self._enqueue_service_response(request)
            return

        joint_name = request.get("args", {}).get("joint_name")
        delay = self._delays.get(joint_name, 0.0) if joint_name else 0.0
        if delay:

            def _respond() -> None:
                time.sleep(delay)
                self._enqueue_service_response(request)

            threading.Thread(target=_respond, daemon=True).start()
        else:
            self._enqueue_service_response(request)

    def _enqueue_joint_states_publish(self) -> None:
        motors = full_motor_fixture()
        now = time.time()
        names = []
        positions = []
        for index, motor in enumerate(motors):
            name = motor["name"]
            if name in self._failures:
                continue
            names.append(name)
            centideg = self._positions.get(name, 120 * (index + 1))
            positions.append(math.radians(centideg / 100.0))
        message = {
            "op": "publish",
            "topic": JOINT_STATES_TOPIC,
            "msg": {
                "header": {
                    "stamp": {
                        "sec": int(now),
                        "nanosec": int((now % 1) * 1e9),
                    }
                },
                "name": names,
                "position": positions,
            },
        }
        self._outbound.put(json.dumps(message))

    def _enqueue_service_response(self, request: dict[str, Any]) -> None:
        response = {
            "op": "service_response",
            "id": request["id"],
            "result": True,
            "values": self._values_for_request(request),
        }
        self._outbound.put(json.dumps(response))

    def _values_for_request(self, request: dict[str, Any]) -> dict[str, Any]:
        joint_names = request.get("args", {}).get("joint_names")
        if joint_names is not None:
            results = []
            positions = []
            for name in joint_names:
                if name in self._failures:
                    results.append(
                        {
                            "motor_name": name,
                            "successful": False,
                            "error_code": "position_unavailable",
                            "detail": self._failures[name],
                        }
                    )
                    positions.append(0)
                    continue
                position = self._positions.get(name, 0)
                results.append(
                    {
                        "motor_name": name,
                        "successful": True,
                        "error_code": "",
                        "detail": "",
                    }
                )
                positions.append(position)
            now = time.time()
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

        joint_name = request.get("args", {}).get("joint_name")
        if joint_name and joint_name in self._failures:
            return {"successful": False, "message": self._failures[joint_name]}
        if joint_name and joint_name in self._positions:
            return {"successful": True, "position": self._positions[joint_name]}
        return {"successful": True, "position": 0}

    def recv(self) -> str:
        return self._outbound.get(timeout=30.0)

    def close(self) -> None:
        pass

    _delays: dict[str, float] = {}
    _batch_delay: float = 0.0
    _failures: dict[str, str] = {}
    _positions: dict[str, int] = {}

    @classmethod
    def reset(cls) -> None:
        cls.instances = []
        cls._delays = {}
        cls._batch_delay = 0.0
        cls._failures = {}
        cls._positions = {}


@pytest.fixture
def fake_rosbridge(monkeypatch):
    _FakeRosbridgeSocket.reset()
    import websocket

    def _create_connection(url: str, timeout: float | None = None):
        return _FakeRosbridgeSocket()

    monkeypatch.setattr(websocket, "create_connection", _create_connection)
    yield _FakeRosbridgeSocket


def test_rosbridge_reuses_one_websocket_for_all_joint_reads(fake_rosbridge):
    motors = [{"name": "motor_a"}, {"name": "motor_b"}, {"name": "motor_c"}]
    backend = PibBackend(timeout=5.0)
    backend.get_motors = lambda: list(motors)
    backend._get_all_joint_positions_available = False

    backend._motors_with_live_positions()

    assert len(fake_rosbridge.instances) == 1


def test_slow_joint_does_not_serialise_other_motor_reads(fake_rosbridge):
    names = [motor["name"] for motor in full_motor_fixture()]
    fake_rosbridge._delays = {names[0]: 0.35}
    for name in names[1:]:
        fake_rosbridge._delays[name] = 0.04

    backend = PibBackend(timeout=10.0)
    backend.get_motors = full_motor_fixture
    backend._get_all_joint_positions_available = False

    started = time.monotonic()
    backend._motors_with_live_positions()
    elapsed = time.monotonic() - started

    # Serial: 0.35s + (MOTOR_COUNT - 1) * 0.04s ≈ 1.27s for 24 motors.
    assert elapsed < 0.85


def test_failed_joint_sets_position_error_without_failing_tool(fake_rosbridge):
    motors = full_motor_fixture()
    failing = motors[3]["name"]
    fake_rosbridge._failures = {failing: "joint offline"}

    backend = PibBackend(timeout=5.0)
    backend.get_motors = lambda: list(motors)
    backend._get_all_joint_positions_available = False

    enriched = backend._motors_with_live_positions()
    by_name = {motor["name"]: motor for motor in enriched}
    assert by_name[failing]["positionError"] == "joint offline"
    assert by_name[failing]["currentPosition"] is None
    assert "positionError" not in by_name[motors[0]["name"]]

    compact = backend.list_motors(detail=False)
    compact_by_name = {motor["name"]: motor for motor in compact["motors"]}
    assert compact_by_name[failing]["positionError"] == "joint offline"


def test_list_motors_default_payload_under_3500_bytes(fake_rosbridge):
    backend = PibBackend(timeout=5.0)
    backend.get_motors = full_motor_fixture

    payload = backend.list_motors(detail=False)
    assert len(payload["motors"]) == MOTOR_COUNT
    for motor in payload["motors"]:
        assert set(motor.keys()) <= {
            "id",
            "name",
            "position",
            "positionError",
            "positionSource",
            "positionAgeSec",
        }

    byte_length = payload_utf8_byte_length(payload)
    assert byte_length < 3500


def test_compact_motor_includes_position_error_key():
    motor = {
        "name": "turn_head_motor",
        "currentPosition": 120,
        "positionError": "unavailable",
    }
    compact = _compact_motor(motor)
    assert compact["positionError"] == "unavailable"
    assert set(compact.keys()) == {"id", "name", "position", "positionError"}
