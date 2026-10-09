#!/usr/bin/env python3
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from button_service.srv import ReadButton, SetButtonColor, WaitForButton
from button_service_node_pkg.button_resolution import ResolvedButton, resolve_buttons
from pib_api_client import bricklet_client

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton

CONNECTED_BRICKLET_ATTEMPTS = 30
CONNECTED_BRICKLET_RETRY_SECONDS = 2


@dataclass
class ButtonRuntime:
    uid: str
    device: BrickletRGBLEDButton
    pressed: bool = False
    switched_on: bool = False
    last_state: Optional[int] = None


class TinkerforgeButtonService(Node):
    """
    Service wrapper for up to three Tinkerforge RGB LED Button Bricklets.

    Button UIDs come from GET /bricklet/connected. Entries are kept when their
    name is an RGB LED Button Bricklet and then ordered by port, which is how
    Cerebra numbers them: button_id 1 is the first port, 2 the second, 3 the
    third. A missing device or a UID the button API rejects is logged once and
    leaves the other buttons in service.

    Configuration:
      TF_HOST=localhost
      TF_PORT=4223
      FLASK_API_BASE_URL=http://flask-app:5000
    """

    def __init__(self) -> None:
        super().__init__("tinkerforge_button_service")

        self.host = os.getenv("TF_HOST", "localhost")
        self.port = int(os.getenv("TF_PORT", "4223"))

        self.buttons: Dict[int, ButtonRuntime] = {}
        self.unavailable: Dict[int, str] = {}
        self.lock = threading.RLock()
        self.state_changed = threading.Condition(self.lock)
        self.ipcon = None

        # A bad device used to escape from here, before the services existed,
        # so one relay in the wrong slot took every button down with it.
        try:
            self.ipcon = IPConnection()
            self._connect_buttons(resolve_buttons(self._load_connected_devices()))
        except Exception as exc:
            self.get_logger().error(
                f"button setup failed: {exc} - buttons stay unavailable"
            )

        self.create_service(
            SetButtonColor, "/tf_button/set_color", self.handle_set_color
        )
        self.create_service(ReadButton, "/tf_button/read", self.handle_read)
        self.create_service(WaitForButton, "/tf_button/wait", self.handle_wait)

        self.get_logger().info("Tinkerforge RGB LED Button services available.")

    def _load_connected_devices(self) -> list:
        """Read GET /bricklet/connected, retrying while pib-api starts.

        Failure is not fatal: the services still come up and each button
        reports that it has no device.
        """
        last_error = "pib-api did not return connected bricklets"
        for attempt in range(1, CONNECTED_BRICKLET_ATTEMPTS + 1):
            try:
                successful, payload = bricklet_client.get_connected_bricklets()
            except Exception as exc:
                successful, payload = False, None
                last_error = exc
            else:
                if successful and isinstance(payload, dict):
                    devices = payload.get("bricklets") or []
                    if isinstance(devices, list):
                        return devices
                last_error = "pib-api did not return connected bricklets"

            self.get_logger().warn(
                f"Could not load connected bricklets "
                f"(attempt {attempt}/{CONNECTED_BRICKLET_ATTEMPTS}): {last_error}"
            )
            if attempt < CONNECTED_BRICKLET_ATTEMPTS:
                time.sleep(CONNECTED_BRICKLET_RETRY_SECONDS)

        self.get_logger().error(
            f"Could not load connected bricklets after {CONNECTED_BRICKLET_ATTEMPTS} "
            f"attempts: {last_error} - buttons stay unavailable"
        )
        return []

    def _connect_buttons(self, resolved: Dict[int, ResolvedButton]) -> None:
        self.get_logger().info(
            f"Connecting to Tinkerforge brickd at {self.host}:{self.port}"
        )
        try:
            self.ipcon.connect(self.host, self.port)
        except Exception as exc:
            reason = f"could not connect to brickd at {self.host}:{self.port}: {exc}"
            self.get_logger().error(
                f"skipping buttons: {reason} - buttons stay unavailable"
            )
            for button_id, slot in resolved.items():
                self.unavailable[button_id] = slot.reason or reason
            return

        for button_id in (1, 2, 3):
            slot = resolved[button_id]
            if not slot.uid:
                reason = slot.reason or "no RGB LED Button Bricklet connected"
                self.unavailable[button_id] = reason
                self.get_logger().error(
                    f"skipping button {button_id}: {reason} - "
                    "this button stays unavailable"
                )
                continue
            try:
                self._register_button(button_id, slot.uid)
            except Exception as exc:
                reason = str(exc) or type(exc).__name__
                self.unavailable[button_id] = reason
                self.get_logger().error(
                    f"skipping button {button_id} '{slot.uid}': {reason} - "
                    "this button stays unavailable"
                )

    def _register_button(self, button_id: int, uid: str) -> None:
        device = BrickletRGBLEDButton(uid, self.ipcon)
        runtime = ButtonRuntime(uid=uid, device=device)

        state = device.get_button_state()
        runtime.last_state = state
        runtime.pressed = state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED

        self.buttons[button_id] = runtime

        def make_callback(current_button_id: int):
            def callback(state_value: int) -> None:
                self._on_button_state_changed(current_button_id, state_value)

            return callback

        device.register_callback(
            device.CALLBACK_BUTTON_STATE_CHANGED,
            make_callback(button_id),
        )

        self.get_logger().info(f"Button {button_id} registered with UID {uid}")

    def _on_button_state_changed(self, button_id: int, state: int) -> None:
        with self.state_changed:
            runtime = self.buttons.get(button_id)
            if runtime is None:
                return

            runtime.last_state = state
            runtime.pressed = state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED

            if runtime.pressed:
                runtime.switched_on = not runtime.switched_on

            self.state_changed.notify_all()

    def _button(self, button_id: int) -> Optional[ButtonRuntime]:
        return self.buttons.get(int(button_id))

    def _unavailable(self, button_id, response):
        try:
            numeric_id = int(button_id)
        except (TypeError, ValueError):
            numeric_id = None
        if numeric_id not in (1, 2, 3):
            response.success = False
            response.message = f"Unknown button_id {button_id}. Valid ids are 1, 2, 3."
            return response
        reason = self.unavailable.get(
            numeric_id, "no RGB LED Button Bricklet connected"
        )
        response.success = False
        response.message = f"Button {numeric_id} is unavailable: {reason}"
        return response

    @staticmethod
    def _color(value: int) -> int:
        return max(0, min(255, int(value)))

    def handle_set_color(self, request, response):
        runtime = self._button(request.button_id)
        if runtime is None:
            return self._unavailable(request.button_id, response)

        try:
            runtime.device.set_color(
                self._color(request.red),
                self._color(request.green),
                self._color(request.blue),
            )
            response.success = True
            response.message = "OK"
        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def handle_read(self, request, response):
        runtime = self._button(request.button_id)
        if runtime is None:
            return self._unavailable(request.button_id, response)

        try:
            state = runtime.device.get_button_state()
            with self.lock:
                runtime.last_state = state
                runtime.pressed = state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED
                response.pressed = runtime.pressed
                response.switched_on = runtime.switched_on

            response.success = True
            response.message = "OK"
        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def handle_wait(self, request, response):
        runtime = self._button(request.button_id)
        if runtime is None:
            return self._unavailable(request.button_id, response)

        timeout_sec = float(request.timeout_sec)
        deadline = None if timeout_sec <= 0 else time.monotonic() + timeout_sec

        try:
            runtime.device.set_color(
                self._color(request.red),
                self._color(request.green),
                self._color(request.blue),
            )

            with self.state_changed:
                previous_switch_state = runtime.switched_on

                while True:
                    if request.switch_mode:
                        if runtime.switched_on != previous_switch_state:
                            response.success = True
                            response.pressed = runtime.pressed
                            response.switched_on = runtime.switched_on
                            response.message = "OK"
                            return response
                    else:
                        if runtime.pressed:
                            response.success = True
                            response.pressed = True
                            response.switched_on = runtime.switched_on
                            response.message = "OK"
                            return response

                    if deadline is None:
                        self.state_changed.wait()
                    else:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            response.success = False
                            response.pressed = runtime.pressed
                            response.switched_on = runtime.switched_on
                            response.message = "Timeout"
                            return response
                        self.state_changed.wait(timeout=remaining)

        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TinkerforgeButtonService()

    try:
        rclpy.spin(node)
    finally:
        try:
            if node.ipcon is not None:
                node.ipcon.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
