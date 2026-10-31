"""Unit tests for the Feetech bus writer priority queue (PR-2138)."""

from __future__ import annotations

import sys
import threading
from contextlib import nullcontext
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
for path in (str(REPO_ROOT / "pib_api" / "client"), str(PIB_MOTORS_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from pib_motors.bus_writer import (  # noqa: E402
    BusWriter,
    reset_bus_writer_registry_for_tests,
)
from pib_motors.stservo_sdk.stservo_def import COMM_SUCCESS  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def monotonic(self) -> float:
        return self.t

    def advance(self, delta: float) -> None:
        self.t += delta


class FakeGroupSyncWrite:
    def __init__(self, owner: FakePk) -> None:
        self._owner = owner
        self.data_dict: dict[int, list[int]] = {}

    def clearParam(self) -> None:
        self.data_dict.clear()

    def txPacket(self) -> int:
        for servo_id, _packet in self.data_dict.items():
            self._owner.writes.append(("sync", servo_id, self._owner.stream_seq))
            self._owner.stream_seq += 1
        return COMM_SUCCESS


class FakePk:
    def __init__(self) -> None:
        self.writes: list[tuple[str, int, int, int]] = []
        self.stream_seq = 0
        self.hold = threading.Event()
        self.hold.set()
        self.groupSyncWrite = FakeGroupSyncWrite(self)

    def WritePosEx(self, servo_id: int, position: int, speed: int, acc: int):
        self.hold.wait(timeout=2.0)
        self.stream_seq += 1
        self.writes.append(("write", servo_id, self.stream_seq, position))
        return COMM_SUCCESS, 0

    def SyncWritePosEx(self, servo_id: int, position: int, speed: int, acc: int):
        self.groupSyncWrite.data_dict[servo_id] = [acc, position, speed]
        return True


@pytest.fixture(autouse=True)
def _clean_registry():
    reset_bus_writer_registry_for_tests()
    yield
    reset_bus_writer_registry_for_tests()


def _writer(clock: FakeClock | None = None) -> tuple[BusWriter, FakePk, FakeClock]:
    fake_clock = clock or FakeClock()
    pk = FakePk()
    lock = threading.Lock()
    writer = BusWriter(
        "test-port",
        ph=object(),
        pk=pk,
        lock=lock,
        clock_fn=fake_clock.monotonic,
        sleep_fn=lambda _s: None,
    )
    writer._bus_access = lambda: nullcontext()  # type: ignore[method-assign]
    return writer, pk, fake_clock


def test_service_runs_before_queued_stream() -> None:
    writer, pk, _clock = _writer()
    order: list[str] = []
    original_write = pk.WritePosEx

    def _tracked_write(servo_id: int, position: int, speed: int, acc: int):
        order.append("stream")
        return original_write(servo_id, position, speed, acc)

    pk.WritePosEx = _tracked_write  # type: ignore[method-assign]
    writer.submit_stream(1, 2048, 1000, 50)

    def _service(_pk: FakePk) -> bool:
        order.append("service")
        return True

    result, error = writer.run_service(_service, wait_timeout=2.0)
    assert error is None
    assert result is True
    assert writer.flush_idle(timeout=2.0)
    assert order == ["service", "stream"]


def test_stream_coalesces_per_pin() -> None:
    writer, pk, _clock = _writer()
    for _ in range(5):
        writer.submit_stream(3, 2100, 500, 40)
    assert writer.counters().coalesced_stream == 4
    assert writer.flush_idle(timeout=2.0)
    assert len(pk.writes) == 1


def test_expired_stream_is_dropped() -> None:
    writer, pk, clock = _writer(clock=FakeClock())
    writer.submit_stream(2, 2048, 1000, 50, deadline=clock.monotonic() + 0.05)
    clock.advance(0.2)
    assert writer.flush_idle(timeout=2.0)
    assert pk.writes == []
    assert writer.counters().dropped_stream >= 1


def test_monotonic_stream_progress_under_load() -> None:
    writer, pk, _clock = _writer()
    for index in range(100):
        writer.submit_stream(7, 2048 + index, 800, 30)
    assert writer.counters().coalesced_stream == 99
    assert writer.flush_idle(timeout=2.0)
    assert len(pk.writes) == 1
    assert pk.writes[0][3] == 2048 + 99


def test_service_deadline_returns_error_to_caller() -> None:
    writer, _pk, clock = _writer(clock=FakeClock())
    clock.advance(5.0)
    result, error = writer.run_service(
        lambda _pk: True,
        deadline=clock.monotonic() - 0.01,
        wait_timeout=1.0,
    )
    assert result is None
    assert error is not None
    assert "deadline expired" in error
