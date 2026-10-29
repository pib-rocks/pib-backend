"""Atomic Feetech bus sweep for joint state publishing (PR-2112, PR-2128).

Two read strategies live here, both under the port's own bus lock:

* the fast one - a single ``GroupSyncRead`` request per port that every servo
  answers in ID order (0.4 s measured for a 26-servo robot);
* the slow one - one addressed transaction per servo (``ReadPosSpeed``), which
  is what the Waveshare CDC-ACM adapters actually answer. Measured on the
  pib5advanced at 192.168.1.168: the group read returned no measurement for any
  servo while single reads returned real positions.

A port that yields nothing from the group read is retried with single reads in
the same tick and remembered, so later sweeps go straight to the slow path
instead of paying a failed broadcast every time.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any

from pib_motors.feetech_actuator import (
    FeetechStSerialActuator,
    _BusAccess,
    ticks_to_centideg,
)
from pib_motors.stservo_sdk.group_sync_read import GroupSyncRead
from pib_motors.stservo_sdk.sts import STS_PRESENT_POSITION_L
from pib_motors.stservo_sdk.stservo_def import COMM_SUCCESS

_LOG = logging.getLogger(__name__)
_NO_BUS_LOG_INTERVAL_SEC = 60.0
_last_no_bus_log_monotonic = 0.0

MODE_GROUP = "group"
MODE_SINGLE = "single"

#: Ports whose group read returns nothing; they are read servo by servo instead.
#: Kept for the life of the process - an adapter does not start answering a
#: broadcast because time passed - and re-evaluated via ``forget_single_read_port``.
_SINGLE_READ_PORTS: set[str] = set()


@dataclass(frozen=True)
class JointStateSweepResult:
    """Measured centidegree positions from one or more port-level sweeps."""

    positions_centideg: dict[str, int]
    sweep_monotonic: float
    ports_attempted: int
    ports_succeeded: int
    #: Which strategy each port needed, keyed by port uid ("group" / "single").
    mode_by_port: dict[str, str] = field(default_factory=dict)
    #: Wall time spent reading each port, for the latency work.
    port_durations_ms: dict[str, float] = field(default_factory=dict)


def _feetech_entries(motors: list[Any]) -> list[tuple[str, FeetechStSerialActuator]]:
    entries: list[tuple[str, FeetechStSerialActuator]] = []
    for motor in motors:
        for actuator in motor.actuators:
            if getattr(actuator, "kind", None) == FeetechStSerialActuator.kind:
                entries.append((motor.name, actuator))
    return entries


def _group_by_port(
    entries: list[tuple[str, FeetechStSerialActuator]],
) -> dict[str, list[tuple[str, FeetechStSerialActuator]]]:
    grouped: dict[str, list[tuple[str, FeetechStSerialActuator]]] = {}
    for motor_name, actuator in entries:
        grouped.setdefault(actuator.uid, []).append((motor_name, actuator))
    return grouped


def _to_centideg(pk: Any, actuator: FeetechStSerialActuator, raw_ticks: Any) -> int:
    host_ticks = pk.sts_tohost(raw_ticks, 15)
    centideg = ticks_to_centideg(int(host_ticks))
    return int(round(centideg * actuator.direction_sign))


def _read_group(
    pk: Any, group: list[tuple[str, FeetechStSerialActuator]]
) -> dict[str, int]:
    """One broadcast request for the whole port; empty when nothing answers."""
    positions: dict[str, int] = {}
    # The packet handler, not the port handler: syncReadTx/syncReadRx live on
    # protocol_packet_handler (the port only moves bytes). Passing the port made
    # every broadcast raise AttributeError, which read like "no measurement".
    group_read = GroupSyncRead(pk, STS_PRESENT_POSITION_L, 2)
    for _, actuator in group:
        group_read.addParam(actuator.pin)
    comm = group_read.txRxPacket()
    if comm != COMM_SUCCESS and not group_read.last_result:
        return positions
    for motor_name, actuator in group:
        available, _error = group_read.isAvailable(
            actuator.pin, STS_PRESENT_POSITION_L, 2
        )
        if not available:
            continue
        raw_ticks = group_read.getData(actuator.pin, STS_PRESENT_POSITION_L, 2)
        positions[motor_name] = _to_centideg(pk, actuator, raw_ticks)
    return positions


def _read_single(
    pk: Any, group: list[tuple[str, FeetechStSerialActuator]]
) -> dict[str, int]:
    """One addressed transaction per servo. The caller already holds the bus."""
    positions: dict[str, int] = {}
    for motor_name, actuator in group:
        try:
            raw_ticks, _speed, result, _error = pk.ReadPosSpeed(actuator.pin)
        except Exception as error:  # noqa: BLE001 - one servo must not stop the port
            _LOG.debug("single read failed for %s: %s", motor_name, error)
            continue
        if result != COMM_SUCCESS:
            continue
        positions[motor_name] = _to_centideg(pk, actuator, raw_ticks)
    return positions


def _sweep_port(
    group: list[tuple[str, FeetechStSerialActuator]],
) -> tuple[dict[str, int], str]:
    """Read one port. Returns the positions and the strategy that produced them."""
    sample = group[0][1]
    ph = sample._ph
    pk = sample._pk
    lock = sample._lock
    if ph is None or pk is None or lock is None:
        return {}, ""

    port_key = str(getattr(sample, "uid", "") or "")
    try:
        with _BusAccess(ph, lock):
            if port_key not in _SINGLE_READ_PORTS:
                # A broadcast may answer nothing, or throw on an adapter that
                # does not implement it. Both mean the same thing here: this
                # port has to be read servo by servo, inside the same lock.
                positions: dict[str, int] = {}
                try:
                    positions = _read_group(pk, group)
                except Exception as error:  # noqa: BLE001
                    _LOG.warning(
                        "joint state sweep: group sync read raised on port %s (%s: %s)",
                        port_key,
                        type(error).__name__,
                        error,
                    )
                if positions:
                    return positions, MODE_GROUP
                if not port_key:
                    return {}, ""
                # Warning, not info: a node logger without handlers drops info
                # lines, and this switch is the one fact that explains why the
                # sweep suddenly costs a round trip per servo.
                _LOG.warning(
                    "joint state sweep: port %s answered no group sync read, "
                    "reading its %d servos individually from now on",
                    port_key,
                    len(group),
                )
            positions = _read_single(pk, group)
            if positions and port_key:
                _SINGLE_READ_PORTS.add(port_key)
            return positions, MODE_SINGLE
    except Exception as error:  # noqa: BLE001 - a dead port must not break the sweep
        _LOG.warning("joint state sweep: port %s failed entirely: %s", port_key, error)
        return {}, ""


def forget_single_read_port(port_key: str) -> None:
    """Drop the memory for one port (used when an adapter is re-plugged)."""
    _SINGLE_READ_PORTS.discard(port_key)


def single_read_ports() -> set[str]:
    """Ports currently known to need individual reads (a copy, for diagnostics)."""
    return set(_SINGLE_READ_PORTS)


def sweep_feetech_joint_positions(motors: list[Any]) -> JointStateSweepResult:
    """Read present position for all connected Feetech actuators, port by port."""
    global _last_no_bus_log_monotonic

    entries = _feetech_entries(motors)
    if not entries:
        return JointStateSweepResult({}, time.monotonic(), 0, 0)

    connected = [
        (name, actuator) for name, actuator in entries if actuator.is_connected()
    ]
    if not connected:
        now = time.monotonic()
        if now - _last_no_bus_log_monotonic >= _NO_BUS_LOG_INTERVAL_SEC:
            _last_no_bus_log_monotonic = now
            _LOG.info(
                "joint state sweep: no connected Feetech actuators (bus absent or SSR off)"
            )
        return JointStateSweepResult({}, time.monotonic(), 0, 0)

    grouped = _group_by_port(connected)
    all_positions: dict[str, int] = {}
    ports_attempted = len(grouped)
    ports_succeeded = 0
    mode_by_port: dict[str, str] = {}
    port_durations_ms: dict[str, float] = {}
    for port_key, group in grouped.items():
        started = time.monotonic()
        port_positions, mode = _sweep_port(group)
        port_durations_ms[port_key] = round((time.monotonic() - started) * 1000.0, 1)
        if mode:
            mode_by_port[port_key] = mode
        if port_positions:
            ports_succeeded += 1
            all_positions.update(port_positions)

    return JointStateSweepResult(
        all_positions,
        time.monotonic(),
        ports_attempted,
        ports_succeeded,
        mode_by_port,
        port_durations_ms,
    )
