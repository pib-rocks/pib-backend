"""Robot Framework library bridging E2E tests to ROS2 services and topics."""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Optional

try:
    import rclpy
    from datatypes.srv import ApplyJointTrajectory, GetJointPosition
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    _RCLPY_AVAILABLE = True
except ImportError:  # pragma: no cover - CI without ROS workspace
    _RCLPY_AVAILABLE = False
    Node = object  # type: ignore[misc, assignment]


class ROS2TestLibrary:
    """Keywords for ROS2 assertions in Robot Framework E2E suites."""

    ROBOT_LIBRARY_SCOPE = "GLOBAL"

    def __init__(self) -> None:
        self._node: Optional[Any] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._mock_mode = os.getenv("ROS2_TEST_MOCK", "true").lower() in (
            "1",
            "true",
            "yes",
        )
        self._mock_positions: dict[str, int] = {}

    def ros2_initialize_test_node(self, node_name: str = "robot_framework_test") -> None:
        if self._mock_mode:
            return
        if not _RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not available; set ROS2_TEST_MOCK=true")
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(node_name)
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    def ros2_shutdown_test_node(self) -> None:
        if self._mock_mode or self._node is None:
            return
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._node = None

    def ros2_wait_for_service(self, service_name: str, timeout_sec: float = 10.0) -> bool:
        if self._mock_mode:
            return True
        assert self._node is not None
        client = self._node.create_client(ApplyJointTrajectory, service_name)
        return client.wait_for_service(timeout_sec=timeout_sec)

    def ros2_apply_joint_trajectory(
        self, motor_name: str, position: int, timeout_sec: float = 15.0
    ) -> bool:
        if self._mock_mode:
            clamped = self._clamp_position(motor_name, position)
            self._mock_positions[motor_name] = clamped
            return True
        assert self._node is not None
        client = self._node.create_client(ApplyJointTrajectory, "apply_joint_trajectory")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        request = ApplyJointTrajectory.Request()
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        request.trajectory = JointTrajectory()
        request.trajectory.joint_names = [motor_name]
        request.trajectory.points = [point]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        response = future.result()
        return bool(response.successful)

    def ros2_get_joint_position(self, motor_name: str, timeout_sec: float = 10.0) -> int:
        if self._mock_mode:
            return self._mock_positions.get(motor_name, 0)
        assert self._node is not None
        client = self._node.create_client(GetJointPosition, "get_joint_position")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"get_joint_position unavailable for {timeout_sec}s")
        request = GetJointPosition.Request()
        request.joint_name = motor_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        response = future.result()
        if not response.successful:
            raise RuntimeError(response.message)
        return int(response.position)

    def ros2_assert_position_clamped(
        self, motor_name: str, requested: int, min_val: int, max_val: int
    ) -> None:
        expected = max(min_val, min(max_val, requested))
        if self._mock_mode:
            actual = self._clamp_position(motor_name, requested)
            if actual != expected:
                raise AssertionError(
                    f"mock clamp mismatch for {motor_name}: {actual} != {expected}"
                )
            self._mock_positions[motor_name] = actual
            return
        if not self.ros2_apply_joint_trajectory(motor_name, requested):
            raise AssertionError(f"apply_joint_trajectory failed for {motor_name}")
        time.sleep(0.5)
        actual = self.ros2_get_joint_position(motor_name)
        if actual != expected:
            raise AssertionError(
                f"{motor_name}: expected clamped position {expected}, got {actual}"
            )

    def ros2_assert_no_http_503_from_flask(self, flask_base_url: str) -> None:
        import requests

        endpoints = ("/motor", "/program", "/pose")
        for path in endpoints:
            response = requests.get(f"{flask_base_url}{path}", timeout=5)
            if response.status_code == 503:
                raise AssertionError(
                    f"Flask must not return 503 for ROS timeouts; got 503 on {path}"
                )

    def _clamp_position(self, motor_name: str, position: int) -> int:
        global_min, global_max = -9000, 9000
        if motor_name == "tilt_forward_motor":
            motor_min, motor_max = -4500, 4500
        else:
            motor_min, motor_max = global_min, global_max
        clamped = max(global_min, min(global_max, position))
        clamped = max(motor_min, min(motor_max, clamped))
        return clamped
