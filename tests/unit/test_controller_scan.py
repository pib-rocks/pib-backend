"""Unit tests for controller_scan helpers (no ROS runtime required)."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOTORS_PKG_DIR = REPO_ROOT / "ros_packages" / "motors"
API_CLIENT_DIR = REPO_ROOT / "pib_api" / "client"
PIB_MOTORS_DIR = MOTORS_PKG_DIR / "pib_motors"

for path in (
    str(API_CLIENT_DIR),
    str(MOTORS_PKG_DIR),
    str(PIB_MOTORS_DIR),
):
    if path not in sys.path:
        sys.path.insert(0, path)

from motors import controller_scan  # noqa: E402


def _controller_two_motors_payload():
    channels = [16, 17, 18, 19, 20]
    return {
        "motors": [
            {
                "name": f"motor_{ch}",
                "channel": ch,
                "controller": {"kind": "feetech_st_serial", "number": 2},
            }
            for ch in channels
        ]
    }


def test_expected_servo_ids_from_motor_api(monkeypatch):
    monkeypatch.setattr(
        controller_scan.motor_client,
        "get_all_motors",
        lambda: (True, _controller_two_motors_payload()),
    )
    assert controller_scan._expected_servo_ids() == {2: [16, 17, 18, 19, 20]}


def test_expected_servo_ids_empty_when_api_fails(monkeypatch):
    monkeypatch.setattr(
        controller_scan.motor_client,
        "get_all_motors",
        lambda: (False, None),
    )
    assert controller_scan._expected_servo_ids() == {}


def test_build_detected_assignments_picks_best_controller():
    serials = ["ADAPTER_A"]
    expected = {1: [1, 2], 2: [16, 17, 18]}
    counts = {1: 1, 2: 3}

    def fake_count(_serial, _expected):
        return counts

    assignments = controller_scan.build_detected_assignments(
        serials, expected, count_matches_fn=fake_count
    )
    assert assignments == [
        {
            "serial": "ADAPTER_A",
            "controllerNumber": 2,
            "matchCount": 3,
            "expectedCount": 3,
        }
    ]


def test_build_detected_assignments_no_controller_below_min_match():
    assignments = controller_scan.build_detected_assignments(
        ["ADAPTER_A"],
        {1: [1, 2]},
        count_matches_fn=lambda _s, _e: {1: 0},
    )
    assert assignments[0]["controllerNumber"] is None
    assert assignments[0]["matchCount"] == 0
    assert assignments[0]["expectedCount"] == 0


def test_count_matches_takes_the_port_per_probe() -> None:
    """Holding the port across all probes blocked the sweep for about a second.

    Each probe must be its own port acquisition, so an adapter is never held longer than
    one response timeout - a servo that is not on this adapter costs exactly that.
    """
    acquisitions: list[str] = []

    class _Access:
        def __init__(self, _ph, _lock) -> None:
            pass

        def __enter__(self) -> None:
            acquisitions.append("enter")

        def __exit__(self, *_args) -> None:
            acquisitions.append("exit")

    class _Pk:
        def ReadPosSpeed(self, servo_id: int):
            return (0, 0, 0, 0) if servo_id == 16 else (-1, 0, 1, "no answer")

    with mock.patch.object(
        controller_scan, "_get_or_open_port", return_value=(None, _Pk(), None)
    ):
        with mock.patch.object(controller_scan, "_BusAccess", _Access):
            counts = controller_scan._count_matches("test-serial", {2: [16, 17, 18]})

    assert counts == {2: 1}
    assert acquisitions == ["enter", "exit"] * 3, acquisitions


def test_present_servos_give_the_match_counts_without_a_scan() -> None:
    """The periodic picture comes from /joint_states, so it must not touch the bus."""
    expected = {1: [1, 2, 3], 2: [16, 17]}
    with mock.patch.object(
        controller_scan,
        "_controller_number_by_serial",
        return_value={"serial-a": 1, "serial-b": 2},
    ):
        with mock.patch.object(controller_scan, "_count_matches") as count_matches:
            assignments = controller_scan.assignments_from_present_servos(
                ["serial-a", "serial-b"], expected, {1, 2, 16}
            )
    count_matches.assert_not_called()
    assert assignments == [
        {
            "serial": "serial-a",
            "controllerNumber": 1,
            "matchCount": 2,
            "expectedCount": 3,
        },
        {
            "serial": "serial-b",
            "controllerNumber": 2,
            "matchCount": 1,
            "expectedCount": 2,
        },
    ]


def test_unassigned_adapter_is_reported_as_unassigned() -> None:
    """No stored address and several controllers: do not guess, report None."""
    expected = {1: [1, 2], 2: [16, 17]}
    with mock.patch.object(
        controller_scan, "_controller_number_by_serial", return_value={}
    ):
        assignments = controller_scan.assignments_from_present_servos(
            ["serial-x"], expected, {1, 16}
        )
    assert assignments[0]["controllerNumber"] is None
    assert assignments[0]["matchCount"] == 0
    assert assignments[0]["expectedCount"] == 0
