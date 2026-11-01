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

#: How often the derived status may be recomputed from /joint_states. It can only change
#: when hardware does, and /joint_states arrives ten times per second.
_STATUS_EVALUATION_INTERVAL_SEC = 5.0

#: The API drops a reported scan result after DETECTED_ASSIGNMENTS_TTL_SEC (120 s), so the
#: unchanged picture has to be re-sent well inside that window. Re-sending costs no bus
#: access and no scan - it is the same numbers from /joint_states.
_REPORT_REFRESH_SEC = 60.0
SERVO_MATCH_MIN = 1
_STARTUP_SCAN_DELAY_SEC = 1.0

_LOG = logging.getLogger(__name__)


def _serial_from_by_id_name(entry_name: str) -> str:
    name = re.sub(r"-port\d+.*$", "", entry_name)
    name = re.sub(r"-if\d+.*$", "", name)
    if "_" not in name:
        return name
    return name.rsplit("_", 1)[-1]


def _servo_channel_by_motor_name() -> dict[str, int]:
    """Motor name to servo channel, so /joint_states names can be counted.

    /joint_states publishes motor names, not channel numbers; without this mapping the
    match count would always be zero.
    """
    ok, response = motor_client.get_all_motors()
    if not ok or not isinstance(response, dict):
        _LOG.warning("controller_scan: could not load motors from API")
        return {}
    mapping: dict[str, int] = {}
    for motor in response.get("motors") or []:
        if not isinstance(motor, dict):
            continue
        controller = motor.get("controller")
        if not isinstance(controller, dict):
            continue
        if controller.get("kind") != "feetech_st_serial":
            continue
        name = motor.get("name")
        channel = motor.get("channel")
        if isinstance(name, str) and isinstance(channel, int):
            mapping[name] = channel
    return mapping


def _controller_number_by_serial() -> dict[str, int]:
    """Map the adapter serial to its controller number using the stored address.

    The address lives in the controller table (controller_model.address) and is assigned
    once, when the robot is provisioned. That is what lets the periodic status come from
    the database and the published joint states instead of from bus probing.
    """
    ok, response = controller_client.get_all_controllers()
    if not ok or not isinstance(response, dict):
        _LOG.warning("controller_scan: could not load controllers from API")
        return {}
    mapping: dict[str, int] = {}
    for controller in response.get("controllers") or []:
        if not isinstance(controller, dict):
            continue
        if controller.get("kind") != "feetech_st_serial":
            continue
        address = controller.get("address")
        number = controller.get("number")
        if isinstance(address, str) and address.strip() and isinstance(number, int):
            mapping[address.strip()] = number
    return mapping


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
    """Probe every expected servo id on this adapter, one port acquisition per probe.

    The lock is taken per probe and not once around the whole loop: a servo that is not
    on this adapter costs the full response timeout (about 40 ms), and holding the port
    across all of them blocked the joint state sweep for about a second per adapter -
    measured as BUS_ACCESS_HELD held_ms=1066 caller=controller_scan.py:_count_matches,
    and the sweep then lost its deadline and published an incomplete message (PR-2138).
    """
    try:
        ph, pk, lock = _get_or_open_port(serial)
    except RuntimeError as error:
        _LOG.warning("controller_scan: cannot open port for %s: %s", serial, error)
        return {}
    counts = {number: 0 for number in expected}
    for controller_number, servo_ids in expected.items():
        for servo_id in servo_ids:
            with _BusAccess(ph, lock):
                _, _, res, _err = pk.ReadPosSpeed(servo_id)
            if res == COMM_SUCCESS:
                counts[controller_number] += 1
    return counts


def _controller_number_for_serial(
    serial: str, expected: dict[int, list[int]]
) -> int | None:
    """Fallback when the address is not stored: the only controller we know of.

    With a single controller in the database there is nothing to disambiguate, so the
    adapter has to be that one. With several, an unassigned address stays None and the
    adapter is reported as not assigned instead of being guessed.
    """
    if serial in {""}:
        return None
    if len(expected) == 1:
        return next(iter(expected))
    return None


def present_channels_from_names(
    names: list[str] | tuple[str, ...], name_to_channel: dict[str, int]
) -> set[int]:
    """/joint_states publishes motor names; the expected ids are servo channels.

    This one line used to be inlined in the ROS node, where no unit test could reach it -
    and it was wrong (it compared names against integer ids, so every matchCount was 0).
    """
    return {name_to_channel[name] for name in names if name in name_to_channel}


def assignments_from_present_servos(
    serials: list[str],
    expected: dict[int, list[int]],
    by_serial: dict[str, int],
    present_channels: set[int],
) -> list[dict[str, Any]]:
    """Match counts per adapter from the servo channels that actually answer.

    ``present_channels`` comes from the channels of the names in /joint_states, so this
    function costs no bus traffic and no API call - both mappings are passed in. It cannot
    tell WHICH other controller sits on a wrongly wired adapter (that is what the startup
    scan is for), but it detects a changed picture: matchCount below expectedCount.
    """
    assignments: list[dict[str, Any]] = []
    for serial in serials:
        controller_number = by_serial.get(serial) or _controller_number_for_serial(
            serial, expected
        )
        expected_ids = expected.get(controller_number, []) if controller_number else []
        counted = sum(1 for servo_id in expected_ids if servo_id in present_channels)
        assignments.append(
            {
                "serial": serial,
                "controllerNumber": controller_number,
                "matchCount": counted,
                "expectedCount": len(expected_ids),
            }
        )
    return assignments


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
        """Probe the bus ONCE at startup, then only watch what the sweep already reports.

        The controller-to-adapter assignment cannot change while the robot runs: changing
        it means unplugging hardware, which needs a restart anyway. The old version
        therefore scanned the whole serial bus every 30 s and held each adapter for about
        a second, which starved the joint state sweep and made the motion stutter.

        After the startup scan the node derives its answer from /joint_states - the sweep
        publishes exactly the servo names that answer - so the periodic refresh costs no
        bus transaction at all and reports only when the picture changes.
        """

        def __init__(self) -> None:
            super().__init__("controller_scan")
            self._startup_timer = self.create_timer(
                _STARTUP_SCAN_DELAY_SEC, self._startup_scan
            )
            self._expected: dict[int, list[int]] = {}
            self._name_to_channel: dict[str, int] = {}
            self._by_serial: dict[str, int] = {}
            self._serials: list[str] = []
            self._last_report: list[dict[str, Any]] = []
            self._last_report_at = 0.0
            self._last_evaluation = 0.0
            self._create_subscription_joint_states()
            self.get_logger().info("Now Running CONTROLLER SCAN")

        def _create_subscription_joint_states(self) -> None:
            from sensor_msgs.msg import JointState

            self.create_subscription(
                JointState, "joint_states", self._on_joint_states, 10
            )

        def _startup_scan(self) -> None:
            self.destroy_timer(self._startup_timer)
            self._expected = _expected_servo_ids()
            self._name_to_channel = _servo_channel_by_motor_name()
            self._by_serial = _controller_number_by_serial()
            self._serials = _detected_serials()
            assignments = build_detected_assignments(self._serials, self._expected)
            self._report(assignments)
            self.get_logger().info(
                f"startup scan reported {len(assignments)} adapters, "
                f"{len(self._name_to_channel)} servo names mapped, "
                f"{len(self._by_serial)} addresses resolved"
            )

        def _on_joint_states(self, msg: Any) -> None:
            """Recompute the picture from the published names - no bus, no API call.

            Throttled: /joint_states arrives about ten times per second, and the answer can
            only change when hardware does.
            """
            if not self._expected or not self._name_to_channel or not self._by_serial:
                # An incomplete mapping would report every adapter as unassigned; better to
                # wait for the next message than to publish a wrong picture.
                return
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_evaluation < _STATUS_EVALUATION_INTERVAL_SEC:
                return
            self._last_evaluation = now
            present_channels = present_channels_from_names(
                msg.name, self._name_to_channel
            )
            assignments = assignments_from_present_servos(
                self._serials, self._expected, self._by_serial, present_channels
            )
            # Also re-send an unchanged picture before the API's TTL drops it.
            if (
                assignments != self._last_report
                or now - self._last_report_at > _REPORT_REFRESH_SEC
            ):
                self._report(assignments)

        def _report(self, assignments: list[dict[str, Any]]) -> None:
            self._last_report = assignments
            self._last_report_at = self.get_clock().now().nanoseconds / 1e9
            try:
                controller_client.report_detected_assignments(assignments)
            except Exception as error:
                self.get_logger().warning(
                    f"controller_scan: failed to report assignments: {error}"
                )

    def main(args=None) -> None:
        rclpy.init(args=args)
        node = ControllerScan()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
