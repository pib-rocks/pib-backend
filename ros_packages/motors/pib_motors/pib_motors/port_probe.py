"""One probe per serial port instead of one per servo.

``check_connection`` opens the port and pings a servo. Against a servo that does
not answer that costs the serial packet timeout - 50.4 ms measured on a
pib5advanced with the motor power off. Probing all 26 servos per tick therefore
burned ~1.6 s each tick and pinned two processes at ~90 % CPU for measuring
nothing.

A port that just failed is remembered for ``PORT_PROBE_TTL_SEC``, so a dead bus
costs one timeout per port per TTL window instead of one per servo per tick.
"""

from __future__ import annotations

import logging
import time

_LOG = logging.getLogger(__name__)

#: How long a port that failed its probe is left alone.
PORT_PROBE_TTL_SEC = 5.0

_dead_ports: dict[str, float] = {}


def port_key(actuator) -> str:
    """Identify the port behind an actuator: adapters are addressed by uid."""
    for attribute in ("uid", "port", "address"):
        value = getattr(actuator, attribute, None)
        if value is not None:
            return str(value)
    return ""


def port_answers(actuator, ttl: float = PORT_PROBE_TTL_SEC) -> bool:
    """True when anything answers on this actuator's port.

    Pass the first actuator of a port; the per-servo reads still skip whatever
    does not answer on its own, so nothing is lost by trusting the port.
    """
    key = port_key(actuator)
    now = time.monotonic()
    deadline = _dead_ports.get(key)
    if deadline is not None and now < deadline:
        return False
    try:
        answers = bool(actuator.is_connected())
    except Exception as error:  # noqa: BLE001 - a hard port error is "no answer"
        _LOG.debug("port probe failed on %s: %s", key, error)
        answers = False
    if answers:
        _dead_ports.pop(key, None)
    elif key:
        _dead_ports[key] = now + ttl
    return answers


def forget_dead_ports() -> None:
    """Clear the probe memory (tests, or after re-plugging an adapter)."""
    _dead_ports.clear()


def dead_ports() -> dict[str, float]:
    return dict(_dead_ports)
