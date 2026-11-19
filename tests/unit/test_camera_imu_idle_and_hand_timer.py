"""Unit tests for 5 Hz idle IMU liveness drain and on-demand hand_fast_timer."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timedelta, timezone
import os
import sys
import types
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.dirname(__file__))
import test_stereo_camera_optimization  # noqa: F401

from ros_packages.camera.oak_d_lite import stereo as camera_stereo
from ros_packages.camera.oak_d_lite.imu import (
    ClockOffsetEstimator,
    PublishRateThrottle,
)
from ros_packages.camera.oak_d_lite.stereo import (
    CameraNode,
    HAND_FAST_POLL_PERIOD_SECONDS,
    IMU_IDLE_POLL_PERIOD_SECONDS,
    IMU_POLL_PERIOD_SECONDS,
    IMU_STALE_AFTER_SECONDS,
)


def _bare_camera_node() -> CameraNode:
    with patch.object(CameraNode, "__init__", lambda self: None):
        node = CameraNode()
    node.imu_queue = MagicMock()
    node.imu_available = True
    node.imu_publisher_ = MagicMock()
    node.imu_publisher_.get_subscription_count.return_value = 0
    node._imu_throttle = PublishRateThrottle()
    node._imu_clock_offset = ClockOffsetEstimator()
    node._imu_publish_times = deque(maxlen=100)
    node._imu_last_received_monotonic = None
    node._imu_timer_period = None
    node.imu_timer = None
    node.hand_fast_queue = None
    node.hand_fast_timer = None
    node.get_logger = MagicMock()

    created_timers: list[tuple[float, object, MagicMock]] = []

    def fake_create_timer(period, callback):
        timer = MagicMock()
        timer.period = period
        timer.callback = callback
        created_timers.append((period, callback, timer))
        return timer

    node.create_timer = MagicMock(side_effect=fake_create_timer)
    node.destroy_timer = MagicMock()
    node._test_created_timers = created_timers  # type: ignore[attr-defined]
    return node


def _fake_imu_packet(device_sec: float = 1.0):
    acc = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
    acc.getSequenceNum = MagicMock(return_value=1)
    acc.getTimestamp = MagicMock(
        return_value=datetime.fromtimestamp(1000.0 + device_sec, tz=timezone.utc)
    )
    acc.getTimestampDevice = MagicMock(return_value=timedelta(seconds=device_sec))
    gyro = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return types.SimpleNamespace(acceleroMeter=acc, gyroscope=gyro)


def test_imu_idle_drains_at_5hz_without_publishing_and_switches_to_100hz_on_subscriber():
    assert IMU_IDLE_POLL_PERIOD_SECONDS == 0.2
    assert IMU_IDLE_POLL_PERIOD_SECONDS < IMU_STALE_AFTER_SECONDS

    node = _bare_camera_node()
    node._sync_imu_timer_period(active=node._imu_has_subscribers())
    assert node._imu_timer_period == IMU_IDLE_POLL_PERIOD_SECONDS
    idle_timer = node.imu_timer

    # Idle tick with 0 subscribers: drains queue, updates liveness, publishes 0 messages.
    node.imu_queue.tryGet.side_effect = [
        types.SimpleNamespace(packets=[_fake_imu_packet(0.1)]),
        None,
    ]
    node._process_imu()
    node.imu_publisher_.publish.assert_not_called()
    assert node._imu_last_received_monotonic is not None
    status_idle = node._imu_status()
    assert status_idle["state"] == "present"
    assert status_idle["fps"] == 0.0

    # Subscriber connects -> immediately switches timer to 100 Hz and publishes.
    node.imu_publisher_.get_subscription_count.return_value = 1
    node.imu_queue.tryGet.side_effect = [
        types.SimpleNamespace(packets=[_fake_imu_packet(0.2)]),
        None,
    ]
    node._process_imu()
    idle_timer.cancel.assert_called_once()
    node.destroy_timer.assert_called_with(idle_timer)
    assert node._imu_timer_period == IMU_POLL_PERIOD_SECONDS
    node.imu_publisher_.publish.assert_called_once()
    active_timer = node.imu_timer

    # Subscriber disconnects -> switches back to 5 Hz and clears publish rate window.
    node.imu_publisher_.get_subscription_count.return_value = 0
    node.imu_queue.tryGet.side_effect = [
        types.SimpleNamespace(packets=[_fake_imu_packet(0.3)]),
        None,
    ]
    node._process_imu()
    active_timer.cancel.assert_called_once()
    assert node._imu_timer_period == IMU_IDLE_POLL_PERIOD_SECONDS
    assert node.imu_publisher_.publish.call_count == 1
    assert node._imu_status()["fps"] == 0.0


def test_hand_fast_timer_is_created_on_demand_and_destroyed_when_idle():
    node = _bare_camera_node()

    # Idle at startup: no 30 Hz timer created.
    node._sync_hand_fast_timer()
    assert node.hand_fast_timer is None
    node.create_timer.assert_not_called()

    # Model starts -> 30 Hz timer created once.
    node.hand_fast_queue = MagicMock()
    node._sync_hand_fast_timer()
    assert node.hand_fast_timer is not None
    assert node.hand_fast_timer.period == HAND_FAST_POLL_PERIOD_SECONDS
    assert node.create_timer.call_count == 1
    timer_obj = node.hand_fast_timer

    # Repeated sync while active is idempotent.
    node._sync_hand_fast_timer()
    assert node.create_timer.call_count == 1

    # Model stops -> timer cancelled and destroyed (0 Hz idle).
    node.hand_fast_queue = None
    node._sync_hand_fast_timer()
    timer_obj.cancel.assert_called_once()
    node.destroy_timer.assert_called_with(timer_obj)
    assert node.hand_fast_timer is None
