#!/usr/bin/env python3
import os
import threading
import time
import requests
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from button_service.srv import ReadButton, SetButtonColor, WaitForButton

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton


@dataclass
class ButtonRuntime:
    uid: str
    device: BrickletRGBLEDButton
    pressed: bool = False
    switched_on: bool = False
    last_state: Optional[int] = None


class TinkerforgeButtonService(Node):
    """
    Service wrapper for three Tinkerforge RGB LED Button Bricklets.

    Preferred configuration:
      TF_HOST=localhost
      TF_PORT=4223
      FLASK_API_BASE_URL=http://flask-app:5000
      TF_BUTTON_BRICKLET_NUMBERS=5,6,7

    Fallback configuration:
      TF_BUTTON_UIDS=uid_button_1,uid_button_2,uid_button_3

    Mapping:
      Button 1 -> first configured Bricklet number / UID
      Button 2 -> second configured Bricklet number / UID
      Button 3 -> third configured Bricklet number / UID
    """

    def __init__(self) -> None:
        super().__init__("tinkerforge_button_service")

        self.host = os.getenv("TF_HOST", "localhost")
        self.port = int(os.getenv("TF_PORT", "4223"))
        self.api_base_url = os.getenv(
            "FLASK_API_BASE_URL",
            "http://flask-app:5000",
        ).rstrip("/")

        self.bricklet_numbers = [
            int(number.strip())
            for number in os.getenv("TF_BUTTON_BRICKLET_NUMBERS", "").split(",")
            if number.strip()
        ]

        self.uids = self._load_button_uids()

        self.ipcon = IPConnection()
        self.buttons: Dict[int, ButtonRuntime] = {}
        self.lock = threading.RLock()
        self.state_changed = threading.Condition(self.lock)

        self._connect_buttons()

        self.create_service(SetButtonColor, "/tf_button/set_color", self.handle_set_color)
        self.create_service(ReadButton, "/tf_button/read", self.handle_read)
        self.create_service(WaitForButton, "/tf_button/wait", self.handle_wait)

        self.get_logger().info("Tinkerforge RGB LED Button services available.")

    def _load_button_uids(self):
        if self.bricklet_numbers:
            uids = []

            for bricklet_number in self.bricklet_numbers:
                url = f"{self.api_base_url}/bricklet/{bricklet_number}"
                self.get_logger().info(f"Loading button UID from {url}")

                response = requests.get(url, timeout=5)
                response.raise_for_status()

                uid = response.json().get("uid")
                if not uid:
                    raise RuntimeError(
                        f"Bricklet {bricklet_number} has no UID configured in Cerebra."
                    )

                uids.append(uid)

            return uids

        return [
            uid.strip()
            for uid in os.getenv("TF_BUTTON_UIDS", "").split(",")
            if uid.strip()
        ]

    def _connect_buttons(self) -> None:
        if len(self.uids) != 3:
            self.get_logger().warn(
                "Button service should receive exactly 3 RGB LED Button UIDs. "
                f"Current count: {len(self.uids)}"
            )

        self.get_logger().info(f"Connecting to Tinkerforge brickd at {self.host}:{self.port}")
        self.ipcon.connect(self.host, self.port)

        for button_id, uid in enumerate(self.uids, start=1):
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

    @staticmethod
    def _color(value: int) -> int:
        return max(0, min(255, int(value)))

    def handle_set_color(self, request, response):
        runtime = self._button(request.button_id)
        if runtime is None:
            response.success = False
            response.message = f"Unknown button_id {request.button_id}. Valid ids are 1, 2, 3."
            return response

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
            response.success = False
            response.message = f"Unknown button_id {request.button_id}. Valid ids are 1, 2, 3."
            return response

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
            response.success = False
            response.message = f"Unknown button_id {request.button_id}. Valid ids are 1, 2, 3."
            return response

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
            node.ipcon.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
