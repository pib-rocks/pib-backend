"""Atomic Feetech bus sweep via GroupSyncRead for joint state publishing (PR-2112)."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
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


@dataclass(frozen=True)
class JointStateSweepResult:
    """Measured centidegree positions from one or more port-level group sync reads."""

    positions_centideg: dict[str, int]
    sweep_monotonic: float
    ports_attempted: int
    ports_succeeded: int


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


def _sweep_port(
    group: list[tuple[str, FeetechStSerialActuator]],
) -> dict[str, int]:
    sample = group[0][1]
    ph = sample._ph
    pk = sample._pk
    lock = sample._lock
    if ph is None or pk is None or lock is None:
        return {}

    positions: dict[str, int] = {}
    try:
        with _BusAccess(ph, lock):
            group_read = GroupSyncRead(ph, STS_PRESENT_POSITION_L, 2)
            for _, actuator in group:
                group_read.addParam(actuator.pin)
            comm = group_read.txRxPacket()
            if comm != COMM_SUCCESS and not group_read.last_result:
                return {}
            for motor_name, actuator in group:
                available, _error = group_read.isAvailable(
                    actuator.pin, STS_PRESENT_POSITION_L, 2
                )
                if not available:
                    continue
                raw_ticks = group_read.getData(actuator.pin, STS_PRESENT_POSITION_L, 2)
                host_ticks = pk.sts_tohost(raw_ticks, 15)
                centideg = ticks_to_centideg(int(host_ticks))
                positions[motor_name] = int(round(centideg * actuator.direction_sign))
    except Exception as error:
        _LOG.debug("group sync read failed on port %s: %s", sample.uid, error)
        return {}
    return positions


def sweep_feetech_joint_positions(motors: list[Any]) -> JointStateSweepResult:
    """Read present position for all connected Feetech actuators, grouped by serial port."""
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
    for group in grouped.values():
        port_positions = _sweep_port(group)
        if port_positions:
            ports_succeeded += 1
            all_positions.update(port_positions)

    return JointStateSweepResult(
        all_positions,
        time.monotonic(),
        ports_attempted,
        ports_succeeded,
    )
