"""Single owner thread per Feetech serial port with a priority job queue (PR-2138).

Priority: SERVICE > SWEEP > STREAM. STREAM targets coalesce per (port, pin); expired
STREAM jobs are dropped. Only the writer thread calls into ``PortHandler`` / ``sts``.
Cross-process exclusion stays in ``feetech_actuator._BusAccess`` (Lock + flock).
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Deque, Iterator

from pib_motors.stservo_sdk.stservo_def import COMM_SUCCESS

_LOG = logging.getLogger(__name__)

# Deadlines are the ceiling for how long a ROS callback thread may wait for the bus.
# The acceptance criteria of PR-2137/PR-2138 ask for a service reply under 100 ms and a
# worst case under 250 ms, so the service deadline cannot be two seconds: a healthy bus
# answers in single-digit milliseconds, and the deadline only exists for the broken case.
DEFAULT_SERVICE_DEADLINE_SEC = 0.25
DEFAULT_SWEEP_DEADLINE_SEC = 0.5
DEFAULT_STREAM_DEADLINE_SEC = 0.1

# A healthy transaction on this bus takes well under a millisecond. Anything above this
# is worth a warning line, because a single slow transaction blocks every other user of
# the port for its duration - that is the mechanism the 0.4 to 1 s node stalls come from.
SLOW_TX_MS = 50.0

#: How long the writer waits for the rest of a tick before writing a stream batch.
#: The interpolation loop submits one target per joint, in sequence, so the second joint of
#: the same adapter arrives a few hundred microseconds later. Without this window every
#: submission became its own bus transaction (measured: 140 to 191 per second per port for
#: two joints at 100 Hz, coalesced 0 to 2), which defeats the group write completely.
#: One millisecond is a hundredth of the 100 Hz tick and is not felt by anything.
STREAM_GATHER_SEC = 0.001

_registry_lock = threading.Lock()
_registry: dict[str, BusWriter] = {}


class BusPriority(Enum):
    SERVICE = 0
    SWEEP = 1
    STREAM = 2


@dataclass
class StreamTarget:
    pin: int
    ticks: int
    speed: int
    acc: int
    deadline: float
    submitted_at: float
    seq: int
    label: str = ""


@dataclass
class _ServiceJob:
    deadline: float
    submitted_at: float
    execute: Callable[[Any], Any]
    label: str = ""
    done: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    error: str | None = None


@dataclass
class _SweepJob:
    deadline: float
    submitted_at: float
    execute: Callable[[Any], Any]
    label: str = ""
    done: threading.Event = field(default_factory=threading.Event)
    result: Any = None
    error: str | None = None


@dataclass
class BusWriterCounters:
    txs: int = 0
    txs_window_start: float = 0.0
    txs_per_sec: float = 0.0
    queue_depth: int = 0
    max_queue_wait_ms: float = 0.0
    dropped_stream: int = 0
    coalesced_stream: int = 0
    #: Longest single bus transaction seen in the current window, and how many crossed
    #: SLOW_TX_MS. This is what names a blocking servo instead of blaming the queue.
    max_tx_ms: float = 0.0
    slow_transactions: int = 0
    #: Longest wait to get the port Lock + flock (cross-process guard), in milliseconds.
    max_acquire_wait_ms: float = 0.0

    def snapshot(self) -> dict[str, float | int]:
        # Every key the log line uses has to be here: a missing key used to raise a
        # KeyError inside the stats timer, which killed motor_control on the robot
        # (KeyError: 'max_tx_ms' propagating out of rclpy.spin).
        return {
            "txs_per_sec": round(self.txs_per_sec, 1),
            "queue_depth": int(self.queue_depth),
            "max_queue_wait_ms": round(self.max_queue_wait_ms, 1),
            "dropped_stream": int(self.dropped_stream),
            "coalesced_stream": int(self.coalesced_stream),
            "max_tx_ms": round(self.max_tx_ms, 1),
            "slow_transactions": int(self.slow_transactions),
            "max_acquire_wait_ms": round(self.max_acquire_wait_ms, 1),
        }


class BusWriter:
    """One background thread per resolved serial device path."""

    def __init__(
        self,
        port_key: str,
        ph: Any,
        pk: Any,
        lock: threading.Lock,
        *,
        clock_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        self.port_key = port_key
        self._ph = ph
        self._pk = pk
        self._lock = lock
        self._clock = clock_fn
        self._sleep = sleep_fn
        self._cond = threading.Condition()
        self._service: Deque[_ServiceJob] = deque()
        self._sweep: Deque[_SweepJob] = deque()
        self._stream: dict[int, StreamTarget] = {}
        self._stream_seq = 0
        self._counters = BusWriterCounters(txs_window_start=clock_fn())
        self._stop = threading.Event()
        # True while a job is being executed: flush_idle() must not report idle for a
        # queue that is empty only because the last target is already in flight.
        self._executing = False
        # Daemon on purpose: shutdown_all_bus_writers()/stop() is the graceful path and
        # is called from the node teardown, but a writer that was never stopped (a test,
        # a crash path) must not keep the interpreter alive forever.
        self._thread = threading.Thread(
            target=self._run, name=f"bus_writer:{port_key}", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float | None = 5.0) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=timeout)

    def counters(self) -> BusWriterCounters:
        return self._counters

    def submit_stream(
        self,
        pin: int,
        ticks: int,
        speed: int,
        acc: int,
        deadline: float | None = None,
        label: str = "",
    ) -> None:
        now = self._clock()
        if deadline is None:
            deadline = now + DEFAULT_STREAM_DEADLINE_SEC
        with self._cond:
            existing = self._stream.get(pin)
            if existing is not None:
                self._counters.coalesced_stream += 1
            self._stream_seq += 1
            self._stream[pin] = StreamTarget(
                pin=pin,
                ticks=ticks,
                speed=speed,
                acc=acc,
                deadline=deadline,
                submitted_at=now,
                seq=self._stream_seq,
                label=label,
            )
            self._refresh_queue_depth()
            self._cond.notify()

    def run_service(
        self,
        execute: Callable[[Any], Any],
        *,
        deadline: float | None = None,
        wait_timeout: float | None = None,
        label: str = "",
    ) -> tuple[Any, str | None]:
        """Enqueue SERVICE work and block until done or deadline (ROS-safe timeout)."""
        now = self._clock()
        if deadline is None:
            deadline = now + DEFAULT_SERVICE_DEADLINE_SEC
        if wait_timeout is None:
            wait_timeout = max(0.0, deadline - now)
        job = _ServiceJob(
            deadline=deadline, submitted_at=now, execute=execute, label=label
        )
        with self._cond:
            self._service.append(job)
            self._refresh_queue_depth()
            self._cond.notify()
        if not job.done.wait(timeout=wait_timeout):
            return None, "bus busy: service deadline expired before completion"
        return job.result, job.error

    def run_sweep(
        self,
        execute: Callable[[Any], Any],
        *,
        deadline: float | None = None,
        wait_timeout: float | None = None,
        label: str = "",
    ) -> tuple[Any, str | None]:
        now = self._clock()
        if deadline is None:
            deadline = now + DEFAULT_SWEEP_DEADLINE_SEC
        if wait_timeout is None:
            wait_timeout = max(0.0, deadline - now)
        job = _SweepJob(
            deadline=deadline, submitted_at=now, execute=execute, label=label
        )
        with self._cond:
            self._sweep.append(job)
            self._refresh_queue_depth()
            self._cond.notify()
        if not job.done.wait(timeout=wait_timeout):
            return None, "bus busy: sweep deadline expired before completion"
        return job.result, job.error

    def flush_idle(self, timeout: float = 1.0) -> bool:
        """Tests: wait until SERVICE, SWEEP, and STREAM queues are empty."""
        deadline = self._clock() + timeout
        while self._clock() < deadline:
            with self._cond:
                if not self._has_work_locked():
                    return True
            self._sleep(0.005)
        with self._cond:
            return not self._has_work_locked()

    def _refresh_queue_depth(self) -> None:
        self._counters.queue_depth = (
            len(self._service) + len(self._sweep) + len(self._stream)
        )

    def _record_queue_wait(self, submitted_at: float) -> None:
        wait_ms = (self._clock() - submitted_at) * 1000.0
        if wait_ms > self._counters.max_queue_wait_ms:
            self._counters.max_queue_wait_ms = wait_ms

    def _record_transaction(self, kind: str, label: str, started_at: float) -> float:
        """Record one bus transaction: its duration and the wait to get the port.

        The wait for Lock + flock is read from the actuator module, because that is where
        the cross-process guard lives. Both numbers are what tells a slow servo apart
        from a busy queue.
        """
        tx_ms = (self._clock() - started_at) * 1000.0
        if tx_ms > self._counters.max_tx_ms:
            self._counters.max_tx_ms = tx_ms
        acquire_ms = 0.0
        try:
            from pib_motors.feetech_actuator import last_bus_acquire_wait_ms

            acquire_ms = last_bus_acquire_wait_ms()
        except Exception:  # noqa: BLE001 - instrumentation must never break the bus
            acquire_ms = 0.0
        if acquire_ms > self._counters.max_acquire_wait_ms:
            self._counters.max_acquire_wait_ms = acquire_ms
        if tx_ms > SLOW_TX_MS:
            self._counters.slow_transactions += 1
            _LOG.warning(
                "[PERF_TRACE] BUS_WRITER_SLOW port=%s kind=%s label=%s tx_ms=%.1f acquire_ms=%.1f",
                self.port_key,
                kind,
                label or "-",
                tx_ms,
                acquire_ms,
            )
        return tx_ms

    def _record_tx(self) -> None:
        now = self._clock()
        self._counters.txs += 1
        elapsed = now - self._counters.txs_window_start
        if elapsed >= 1.0:
            self._counters.txs_per_sec = self._counters.txs / elapsed
            self._counters.txs = 0
            self._counters.txs_window_start = now
            self._counters.max_queue_wait_ms = 0.0
            self._counters.max_tx_ms = 0.0
            self._counters.slow_transactions = 0
            self._counters.max_acquire_wait_ms = 0.0

    def _run(self) -> None:
        while not self._stop.is_set():
            batch: list[StreamTarget] | _ServiceJob | _SweepJob | None
            with self._cond:
                while not self._stop.is_set() and not self._has_work_locked():
                    self._cond.wait(timeout=0.05)
                if self._stop.is_set():
                    break
                batch = self._pop_next_locked()
                self._refresh_queue_depth()
                if batch is not None:
                    self._executing = True
            if batch is None:
                continue
            if isinstance(batch, list):
                batch = self._gather_same_tick(batch)
            try:
                self._execute(batch)
            finally:
                with self._cond:
                    self._executing = False
                    self._cond.notify_all()

    def _gather_same_tick(self, batch: list[StreamTarget]) -> list[StreamTarget]:
        """Wait one short window so the rest of the tick joins the same group write.

        Only stream targets are collected; if a SERVICE or SWEEP job is waiting, the batch
        goes out immediately - those are the ones with a caller behind them.
        """
        self._sleep(STREAM_GATHER_SEC)
        with self._cond:
            if self._service or self._sweep or not self._stream:
                return batch
            now = self._clock()
            by_pin = {target.pin: target for target in batch}
            for pin in list(self._stream):
                target = self._stream[pin]
                if target.deadline < now:
                    del self._stream[pin]
                    self._counters.dropped_stream += 1
                    continue
                current = by_pin.get(pin)
                if current is None or target.seq > current.seq:
                    by_pin[pin] = target
                del self._stream[pin]
            self._refresh_queue_depth()
            return sorted(by_pin.values(), key=lambda item: item.seq)

    def _has_work_locked(self) -> bool:
        if self._executing or self._service or self._sweep:
            return True
        return bool(self._stream)

    def _pop_next_locked(self) -> list[StreamTarget] | _ServiceJob | _SweepJob | None:
        if self._service:
            return self._service.popleft()
        if self._sweep:
            return self._sweep.popleft()
        if not self._stream:
            return None
        now = self._clock()
        pending: list[StreamTarget] = []
        expired_pins: list[int] = []
        for pin, target in self._stream.items():
            if target.deadline < now:
                expired_pins.append(pin)
            else:
                pending.append(target)
        for pin in expired_pins:
            del self._stream[pin]
            self._counters.dropped_stream += 1
        if not pending:
            return None
        for target in pending:
            self._stream.pop(target.pin, None)
        pending.sort(key=lambda t: t.seq)
        return pending

    def _execute(self, work: list[StreamTarget] | _ServiceJob | _SweepJob) -> None:
        if isinstance(work, _ServiceJob):
            self._execute_service(work)
        elif isinstance(work, _SweepJob):
            self._execute_sweep(work)
        else:
            self._execute_stream_batch(work)

    def _execute_service(self, job: _ServiceJob) -> None:
        now = self._clock()
        if now > job.deadline:
            job.error = "bus busy: service deadline expired before bus access"
            job.done.set()
            return
        self._record_queue_wait(job.submitted_at)
        started_tx = self._clock()
        try:
            with self._bus_access():
                job.result = job.execute(self._pk)
            self._record_tx()
            self._record_transaction("service", job.label, started_tx)
        except Exception as error:  # noqa: BLE001
            job.error = str(error)
        job.done.set()

    def _execute_sweep(self, job: _SweepJob) -> None:
        now = self._clock()
        if now > job.deadline:
            job.error = "bus busy: sweep deadline expired before bus access"
            job.done.set()
            return
        self._record_queue_wait(job.submitted_at)
        started_tx = self._clock()
        try:
            with self._bus_access():
                job.result = job.execute(self._pk)
            self._record_tx()
            self._record_transaction("sweep", job.label, started_tx)
        except Exception as error:  # noqa: BLE001
            job.error = str(error)
        job.done.set()

    def _bus_access(self) -> Any:
        from pib_motors.feetech_actuator import _BusAccess

        return _BusAccess(self._ph, self._lock)

    def _execute_stream_batch(self, targets: list[StreamTarget]) -> None:
        if not targets:
            return
        self._record_queue_wait(min(t.submitted_at for t in targets))
        started_tx = self._clock()
        label = targets[0].label or ",".join(str(t.pin) for t in targets)
        try:
            with self._bus_access():
                if len(targets) == 1:
                    target = targets[0]
                    res, _err = self._pk.WritePosEx(
                        target.pin, target.ticks, target.speed, target.acc
                    )
                    if res != COMM_SUCCESS:
                        _LOG.debug(
                            "stream WritePosEx failed on %s id %s res=%s",
                            self.port_key,
                            target.pin,
                            res,
                        )
                else:
                    self._pk.groupSyncWrite.clearParam()
                    for target in targets:
                        self._pk.SyncWritePosEx(
                            target.pin, target.ticks, target.speed, target.acc
                        )
                    res = self._pk.groupSyncWrite.txPacket()
                    self._pk.groupSyncWrite.clearParam()
                    if res != COMM_SUCCESS:
                        _LOG.debug(
                            "stream sync write failed on %s res=%s",
                            self.port_key,
                            res,
                        )
            self._record_tx()
            self._record_transaction("stream", f"pins:{label}", started_tx)
        except Exception as error:  # noqa: BLE001
            _LOG.debug("stream batch failed on %s: %s", self.port_key, error)


def get_bus_writer(
    port_key: str,
    ph: Any,
    pk: Any,
    lock: threading.Lock,
    *,
    clock_fn: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> BusWriter:
    with _registry_lock:
        existing = _registry.get(port_key)
        if existing is not None:
            return existing
        writer = BusWriter(port_key, ph, pk, lock, clock_fn=clock_fn, sleep_fn=sleep_fn)
        _registry[port_key] = writer
        return writer


def all_bus_writers() -> Iterator[BusWriter]:
    with _registry_lock:
        yield from list(_registry.values())


def shutdown_all_bus_writers(timeout: float | None = 5.0) -> None:
    with _registry_lock:
        writers = list(_registry.values())
        _registry.clear()
    for writer in writers:
        writer.stop(timeout=timeout)


def format_bus_writer_summary(writer: BusWriter) -> str:
    snap = writer.counters().snapshot()
    return (
        f"[PERF_TRACE] BUS_WRITER port={writer.port_key} "
        f"txs_per_s={snap['txs_per_sec']} "
        f"queue={snap['queue_depth']} "
        f"max_wait_ms={snap['max_queue_wait_ms']} "
        f"dropped_stream={snap['dropped_stream']} "
        f"coalesced={snap['coalesced_stream']} "
        f"max_tx_ms={snap['max_tx_ms']} "
        f"slow={snap['slow_transactions']} "
        f"max_acquire_ms={snap['max_acquire_wait_ms']}"
    )


def reset_bus_writer_registry_for_tests() -> None:
    """Tests only: stop writers and clear registry (call before reusing fake ports)."""
    shutdown_all_bus_writers(timeout=1.0)
