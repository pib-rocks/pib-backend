import time
from typing import Optional

import rclpy
from rclpy.node import Node

from button_service.srv import ReadButton, SetButtonColor, WaitForButton


_node: Optional[Node] = None


def _ensure_node() -> Node:
    global _node
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = rclpy.create_node("pib_button_blockly_client")
    return _node


def _call(service_type, service_name: str, request, timeout_sec: float = 10.0):
    node = _ensure_node()
    client = node.create_client(service_type, service_name)

    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service {service_name} not available")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done():
        raise TimeoutError(f"Service call to {service_name} timed out")

    result = future.result()
    if result is None:
        raise RuntimeError(f"Service call to {service_name} failed")

    if hasattr(result, "success") and not result.success:
        raise RuntimeError(result.message)

    return result


def set_button_color(button_id: int, red: int, green: int, blue: int) -> None:
    req = SetButtonColor.Request()
    req.button_id = int(button_id)
    req.red = int(red)
    req.green = int(green)
    req.blue = int(blue)
    _call(SetButtonColor, "/tf_button/set_color", req)


def is_button_pressed(button_id: int) -> bool:
    req = ReadButton.Request()
    req.button_id = int(button_id)
    res = _call(ReadButton, "/tf_button/read", req)
    return bool(res.pressed)


def is_button_switched_on(button_id: int) -> bool:
    req = ReadButton.Request()
    req.button_id = int(button_id)
    res = _call(ReadButton, "/tf_button/read", req)
    return bool(res.switched_on)


def wait_for_button_press(button_id: int, red: int, green: int, blue: int, timeout_sec: float = 0.0) -> bool:
    req = WaitForButton.Request()
    req.button_id = int(button_id)
    req.red = int(red)
    req.green = int(green)
    req.blue = int(blue)
    req.switch_mode = False
    req.timeout_sec = float(timeout_sec)
    res = _call(WaitForButton, "/tf_button/wait", req, timeout_sec=max(10.0, float(timeout_sec) + 1.0))
    return bool(res.pressed)


def wait_for_button_switch(button_id: int, red: int, green: int, blue: int, timeout_sec: float = 0.0) -> bool:
    req = WaitForButton.Request()
    req.button_id = int(button_id)
    req.red = int(red)
    req.green = int(green)
    req.blue = int(blue)
    req.switch_mode = True
    req.timeout_sec = float(timeout_sec)
    res = _call(WaitForButton, "/tf_button/wait", req, timeout_sec=max(10.0, float(timeout_sec) + 1.0))
    return bool(res.switched_on)
