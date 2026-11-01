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

from pib_motors.port_probe import forget_dead_ports, port_answers
from pib_motors.port_recovery import note_failure, note_success
from pib_motors.feetech_actuator import (
    FeetechStSerialActuator,
    resolve_serial_device,
    ticks_to_centideg,
)
from pib_motors.bus_writer import get_bus_writer
from pib_motors.stservo_sdk.stservo_def import COMM_SUCCESS
from pib_motors.units import centideg_to_rad

_LOG = logging.getLogger(__name__)
_NO_BUS_LOG_INTERVAL_SEC = 60.0
_last_no_bus_log_monotonic = 0.0

MODE_SINGLE = "single"
MODE_UNREACHABLE = "unreachable"


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
    #: Ports whose single probe failed and that were therefore skipped.
    ports_unreachable: int = 0


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


def serial_for_port(actuator: Any) -> str:
    """The device path this actuator lives on, resolved the same way the writer does."""
    return resolve_serial_device(str(getattr(actuator, "uid", "") or ""))


def _port_answers(group: list[tuple[str, FeetechStSerialActuator]]) -> bool:
    """One probe per port instead of one per servo (see ``port_probe``)."""
    return port_answers(group[0][1])


def _note_and_maybe_reopen(device: str) -> None:
    """Feed the failure into the policy and reopen the port once it has had enough."""
    from pib_motors.feetech_actuator import reopen_port

    if note_failure(device):
        reopen_port(device)


def _sweep_port(
    group: list[tuple[str, FeetechStSerialActuator]],
) -> tuple[dict[str, int], str]:
    """Read one port, one addressed transaction per servo."""
    sample = group[0][1]
    ph = sample._ph
    pk = sample._pk
    lock = sample._lock
    if ph is None or pk is None or lock is None:
        return {}, ""

    port_key = str(getattr(sample, "uid", "") or "")
    try:
        device = resolve_serial_device(sample.uid)
        writer = get_bus_writer(device, ph, pk, lock)

        def _execute(bus_pk: Any) -> dict[str, int]:
            return _read_single(bus_pk, group)

        positions, error = writer.run_sweep(
            _execute, label=f"sweep:{len(group)} servos"
        )
        if error:
            _LOG.error("joint state sweep: port %s sweep failed: %s", port_key, error)
            _note_and_maybe_reopen(device)
            return {}, ""
        note_success(device)
        return positions or {}, MODE_SINGLE
    except Exception as error:  # noqa: BLE001 - a dead port must not break the sweep
        _LOG.error("joint state sweep: port %s failed entirely: %s", port_key, error)
        try:
            _note_and_maybe_reopen(resolve_serial_device(sample.uid))
        except (
            Exception
        ) as error:  # noqa: BLE001 - never let the recovery break the sweep
            _LOG.error(
                "joint state sweep: port %s recovery failed: %s",
                port_key,
                error,
                exc_info=True,
            )
        return {}, ""


def sweep_feetech_joint_positions(motors: list[Any]) -> JointStateSweepResult:
    """Read present position for all connected Feetech actuators, port by port."""
    global _last_no_bus_log_monotonic

    entries = _feetech_entries(motors)
    if not entries:
        return JointStateSweepResult({}, time.monotonic(), 0, 0)

    grouped = _group_by_port(entries)
    all_positions: dict[str, int] = {}
    ports_attempted = 0
    ports_succeeded = 0
    ports_unreachable = 0
    mode_by_port: dict[str, str] = {}
    port_durations_ms: dict[str, float] = {}
    for port_key, group in grouped.items():
        if not _port_answers(group):
            ports_unreachable += 1
            mode_by_port[port_key] = MODE_UNREACHABLE
            # A port the probe cannot reach counts as a failure too. Without this the recovery
            # gave up after a single reopen: the probe skipped the port, so no further failure
            # was ever counted and the retry budget was never spent - measured on the robot,
            # ports_ok stayed at 3/4 for a minute with only one PORT_REOPEN in the log.
            try:
                _note_and_maybe_reopen(serial_for_port(group[0][1]))
            except (
                Exception
            ) as error:  # noqa: BLE001 - recovery must never break the sweep
                # Loud on purpose: swallowing this hid why the retry loop never ran on the
                # robot (one PORT_REOPEN in the log, then silence, ports_ok stuck at 3/4).
                _LOG.error(
                    "joint state sweep: port %s recovery failed: %s",
                    port_key,
                    error,
                    exc_info=True,
                )
            continue
        ports_attempted += 1
        started = time.monotonic()
        port_positions, mode = _sweep_port(group)
        port_durations_ms[port_key] = round((time.monotonic() - started) * 1000.0, 1)
        if mode:
            mode_by_port[port_key] = mode
        if port_positions:
            ports_succeeded += 1
            all_positions.update(port_positions)

    if not all_positions and ports_unreachable:
        now = time.monotonic()
        if now - _last_no_bus_log_monotonic >= _NO_BUS_LOG_INTERVAL_SEC:
            _last_no_bus_log_monotonic = now
            _LOG.warning(
                "joint state sweep: %d of %d ports did not answer the probe "
                "(adapters absent or motor power off)",
                ports_unreachable,
                len(grouped),
            )

    return JointStateSweepResult(
        all_positions,
        time.monotonic(),
        ports_attempted,
        ports_succeeded,
        mode_by_port,
        port_durations_ms,
        ports_unreachable,
    )


def fill_joint_state(message: Any, positions_centideg: dict[str, int]) -> int:
    """Write sorted joint names and radian positions into a JointState message.

    The field is ``name`` - ``sensor_msgs/JointState`` has exactly
    ``header, name, position, velocity, effort`` (read from the robot image).
    Writing ``joint_names`` raised AttributeError inside the publish callback and
    killed the whole motor_control process the first time a sweep had data.

    Lives next to the sweep because it publishes exactly what the sweep measured;
    kept free of ROS imports so it stays unit-testable.
    """
    written = 0
    for motor_name in sorted(positions_centideg):
        message.name.append(motor_name)
        message.position.append(centideg_to_rad(positions_centideg[motor_name]))
        written += 1
    return written
