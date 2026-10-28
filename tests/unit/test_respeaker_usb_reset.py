"""Unit tests for ReSpeaker USB reset recovery helpers."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

ROS_AUDIO_PACKAGE = Path(__file__).parents[2] / "ros_packages" / "ros_audio_io"
sys.path.insert(0, str(ROS_AUDIO_PACKAGE))

from ros_audio_io.device_retry import (  # noqa: E402
    RESPEAKER_USB_PID,
    RESPEAKER_USB_VID,
    reset_respeaker_usb_device,
    should_attempt_usb_reset,
)


@pytest.mark.parametrize(
    ("attempt", "reason", "device_name", "expected"),
    [
        (2, "device open failure", "ReSpeaker", False),
        (3, "device open failure", "ReSpeaker 4 Mic Array", True),
        (6, "device busy", "ArrayUAC10", True),
        (3, "no matching input device", "ReSpeaker", False),
        (3, "device open failure", "USB PnP Sound Device", False),
        (3, "device open failure", "", True),
    ],
)
def test_should_attempt_usb_reset(attempt, reason, device_name, expected):
    assert (
        should_attempt_usb_reset(attempt, reason, device_name=device_name) is expected
    )


def test_reset_respeaker_usb_device_success():
    device = MagicMock()
    usb_core = SimpleNamespace(
        find=MagicMock(return_value=device),
        USBError=Exception,
    )
    ok, message = reset_respeaker_usb_device(usb_core_module=usb_core)
    assert ok is True
    assert "reset succeeded" in message
    usb_core.find.assert_called_once_with(
        idVendor=RESPEAKER_USB_VID, idProduct=RESPEAKER_USB_PID
    )
    device.reset.assert_called_once_with()


def test_reset_respeaker_usb_device_not_found():
    usb_core = SimpleNamespace(find=MagicMock(return_value=None), USBError=Exception)
    ok, message = reset_respeaker_usb_device(usb_core_module=usb_core)
    assert ok is False
    assert "not found" in message


def test_reset_respeaker_usb_device_usb_error():
    class USBError(Exception):
        pass

    device = MagicMock()
    device.reset.side_effect = USBError("permission denied")
    usb_core = SimpleNamespace(
        find=MagicMock(return_value=device),
        USBError=USBError,
    )
    ok, message = reset_respeaker_usb_device(usb_core_module=usb_core)
    assert ok is False
    assert "USB reset failed" in message
