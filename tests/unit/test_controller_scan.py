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
