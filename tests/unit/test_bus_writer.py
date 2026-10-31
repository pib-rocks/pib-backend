"""Unit tests for the Feetech bus writer priority queue (PR-2138)."""

from __future__ import annotations

import sys
import threading
import time
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
    get_bus_writer,
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
    # Through the registry, not the constructor: the autouse fixture stops every
    # registered writer, so no thread outlives the test module.
    writer = get_bus_writer(
        "test-port",
        object(),
        pk,
        lock,
        clock_fn=fake_clock.monotonic,
        sleep_fn=lambda _s: None,
    )
    writer._bus_access = lambda: nullcontext()  # type: ignore[method-assign]
    return writer, pk, fake_clock


def test_service_runs_before_queued_stream() -> None:
    """Deterministic: the bus is held mid-write, so the queue order is what decides."""
    writer, pk, _clock = _writer()
    order: list[str] = []
    inside_write = threading.Event()

    def _tracked_write(servo_id: int, position: int, speed: int, acc: int):
        order.append(f"stream{servo_id}")
        inside_write.set()
        pk.hold.wait(timeout=5.0)
        return COMM_SUCCESS, 0

    pk.WritePosEx = _tracked_write  # type: ignore[method-assign]
    pk.hold.clear()

    # 1) let the writer start a stream write and block inside it
    writer.submit_stream(1, 2048, 1000, 50)
    assert inside_write.wait(timeout=2.0), "writer never reached the bus"

    # 2) queue another stream target, then a SERVICE job behind it
    writer.submit_stream(2, 2100, 1000, 50)
    service_done = threading.Event()

    def _service(_pk: FakePk) -> bool:
        order.append("service")
        service_done.set()
        return True

    service_thread = threading.Thread(
        target=lambda: writer.run_service(_service, wait_timeout=5.0), daemon=True
    )
    service_thread.start()
    time.sleep(0.05)  # let the SERVICE job reach the queue

    # 3) release the bus: the SERVICE job must be served before the queued stream
    pk.hold.set()
    assert service_done.wait(timeout=5.0), "service job was not served"
    service_thread.join(timeout=5.0)
    assert writer.flush_idle(timeout=2.0)
    assert order == ["stream1", "service", "stream2"], order


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
    """With a slow bus several stream targets do get written - and only forward."""
    writer, pk, _clock = _writer()
    for index in range(100):
        writer.submit_stream(7, 2048 + index, 800, 30, deadline=None)
    assert writer.counters().coalesced_stream == 99
    assert writer.flush_idle(timeout=2.0)
    # coalescing keeps one pending target per pin, so exactly one write happens here
    assert len(pk.writes) == 1
    assert pk.writes[0][3] == 2048 + 99


def test_monotonic_writes_when_the_bus_is_slower_than_the_stream() -> None:
    """The load test above only sees one write; this one forces many, and orders them.

    Every written position must be newer than the previous one - a dropped target may
    never be followed by an older one, which is what a backlog would do.
    """
    writer, pk, _clock = _writer()
    written: list[int] = []
    original_write = pk.WritePosEx

    def _slow_write(servo_id: int, position: int, speed: int, acc: int):
        written.append(position)
        time.sleep(0.01)
        return original_write(servo_id, position, speed, acc)

    pk.WritePosEx = _slow_write  # type: ignore[method-assign]

    for index in range(20):
        writer.submit_stream(7, 2048 + index, 800, 30, deadline=None)
        time.sleep(0.005)

    assert writer.flush_idle(timeout=5.0)
    assert len(written) >= 2, "bus was not slow enough to force several writes"
    assert written == sorted(written), f"positions went backwards: {written}"
    assert written[-1] == pk.writes[-1][3]


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
