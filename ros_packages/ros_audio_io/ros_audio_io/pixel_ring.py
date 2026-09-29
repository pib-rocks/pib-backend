"""USB commands for the ReSpeaker Mic Array v2.0 pixel ring."""

from __future__ import annotations

from typing import List, Optional

import usb.util


class PixelRing:
    """Drive the XVF3000 LED ring through its vendor control endpoint.

    This driver is intentionally copied from the Flask backend into the ROS
    package: ros-audio-io is now the only USB-device owner, so LED commands must
    share that owner instead of causing the backend to open the device.

    The ring protocol is write-only. A successful USB control transfer is the
    strongest acknowledgement the firmware provides; unlike XVF3000 tuning
    registers, LED state cannot be read back from the device.
    """

    TIMEOUT = 8000

    def __init__(self, dev):
        self.dev = dev

    def write(self, command: int, data: Optional[List[int]] = None) -> None:
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            command,
            0x1C,
            [0] if data is None else data,
            self.TIMEOUT,
        )

    def off(self) -> None:
        self.write(1, [0, 0, 0, 0])

    def mono(self, color: int) -> None:
        self.write(
            1,
            [(color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, 0],
        )

    def listen(self) -> None:
        self.write(2)

    def speak(self) -> None:
        self.write(3)

    def think(self) -> None:
        self.write(4)

    def spin(self) -> None:
        self.write(5)

    def trace(self) -> None:
        self.write(0)

    def set_brightness(self, brightness: int) -> None:
        self.write(0x20, [brightness & 0xFF])

    def set_vad_led(self, enabled: bool) -> None:
        self.write(0x22, [int(enabled)])
