"""Probe Waveshare USB-serial adapters and report likely controller assignments."""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Any

from pib_api_client import controller_client, motor_client
from pib_motors.feetech_actuator import (
    COMM_SUCCESS,
    SERIAL_BY_ID_DIR,
    _BusAccess,
    _get_or_open_port,
)

SCAN_INTERVAL_SEC = 30.0
SERVO_MATCH_MIN = 1
_STARTUP_SCAN_DELAY_SEC = 1.0

_LOG = logging.getLogger(__name__)


def _serial_from_by_id_name(entry_name: str) -> str:
    name = re.sub(r"-port\d+.*$", "", entry_name)
    name = re.sub(r"-if\d+.*$", "", name)
    if "_" not in name:
        return name
    return name.rsplit("_", 1)[-1]


def _expected_servo_ids() -> dict[int, list[int]]:
    ok, response = motor_client.get_all_motors()
    if not ok or not isinstance(response, dict):
        _LOG.warning("controller_scan: could not load motors from API")
        return {}
    expected: dict[int, list[int]] = {}
    for motor in response.get("motors") or []:
        if not isinstance(motor, dict):
            continue
        controller = motor.get("controller")
        if not isinstance(controller, dict):
            continue
        if controller.get("kind") != "feetech_st_serial":
            continue
        controller_number = controller.get("number")
        if not isinstance(controller_number, int):
            continue
        channel = motor.get("channel")
        if channel is None:
            continue
        expected.setdefault(controller_number, []).append(channel)
    return expected


def _detected_serials(serial_dir: Path | None = None) -> list[str]:
    target = serial_dir or SERIAL_BY_ID_DIR
    serials: list[str] = []
    seen: set[str] = set()
    if not target.is_dir():
        return serials
    for entry in sorted(target.iterdir(), key=lambda p: p.name):
        serial = _serial_from_by_id_name(entry.name).strip()
        if not serial or serial in seen:
            continue
        seen.add(serial)
        serials.append(serial)
    return serials


def _count_matches(serial: str, expected: dict[int, list[int]]) -> dict[int, int]:
    try:
        ph, pk, lock = _get_or_open_port(serial)
    except RuntimeError as error:
        _LOG.warning("controller_scan: cannot open port for %s: %s", serial, error)
        return {}
    counts = {number: 0 for number in expected}
    with _BusAccess(ph, lock):
        for controller_number, servo_ids in expected.items():
            for servo_id in servo_ids:
                _, _, res, _err = pk.ReadPosSpeed(servo_id)
                if res == COMM_SUCCESS:
                    counts[controller_number] += 1
    return counts


def build_detected_assignments(
    serials: list[str],
    expected: dict[int, list[int]],
    count_matches_fn=_count_matches,
) -> list[dict[str, Any]]:
    assignments: list[dict[str, Any]] = []
    for serial in serials:
        per_controller = count_matches_fn(serial, expected)
        best_number = None
        best_count = 0
        for controller_number, count in per_controller.items():
            if count > best_count:
                best_count = count
                best_number = controller_number
        controller_number = best_number if best_count >= SERVO_MATCH_MIN else None
        expected_count = len(expected.get(best_number, []))
        assignments.append(
            {
                "serial": serial,
                "controllerNumber": controller_number,
                "matchCount": best_count,
                "expectedCount": expected_count,
            }
        )
    return assignments


try:
    import rclpy
    from rclpy.node import Node
except ImportError:  # pragma: no cover - unit tests without ROS
    rclpy = None
    Node = object  # type: ignore[misc, assignment]


if rclpy is not None:

    class ControllerScan(Node):
        def __init__(self) -> None:
            super().__init__("controller_scan")
            self.create_timer(_STARTUP_SCAN_DELAY_SEC, self._scan)
            self.create_timer(SCAN_INTERVAL_SEC, self._scan)
            self.get_logger().info("Now Running CONTROLLER SCAN")

        def _scan(self) -> None:
            expected = _expected_servo_ids()
            serials = _detected_serials()
            assignments = build_detected_assignments(serials, expected)
            try:
                controller_client.report_detected_assignments(assignments)
            except Exception as error:
                self.get_logger().warning(
                    f"controller_scan: failed to report assignments: {error}"
                )
            self.get_logger().info(f"scan reported {len(assignments)} adapters")

    def main(args=None) -> None:
        rclpy.init(args=args)
        node = ControllerScan()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
