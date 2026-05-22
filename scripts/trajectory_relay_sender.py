#!/usr/bin/env python3
import json
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


def message_to_payload(msg: JointTrajectory) -> dict:
    points = []
    for point in msg.points:
        points.append(
            {
                "positions": list(point.positions),
                "velocities": list(point.velocities),
                "accelerations": list(point.accelerations),
                "effort": list(point.effort),
                "time_from_start": {
                    "sec": int(point.time_from_start.sec),
                    "nanosec": int(point.time_from_start.nanosec),
                },
            }
        )

    return {
        "joint_names": list(msg.joint_names),
        "points": points,
    }


class TrajectoryRelaySender(Node):
    def __init__(self):
        super().__init__("trajectory_relay_sender")

        self.remote_host = os.getenv("TRAJECTORY_RELAY_REMOTE_HOST", "").strip()
        self.remote_port = int(os.getenv("TRAJECTORY_RELAY_REMOTE_PORT", "15555"))

        if not self.remote_host:
            raise RuntimeError("TRAJECTORY_RELAY_REMOTE_HOST is required")

        self._sock = None
        self._sock_lock = threading.Lock()

        self.create_subscription(
            JointTrajectory, "/joint_trajectory", self._on_trajectory, 10
        )
        self.create_timer(2.0, self._ensure_connection)

        self.get_logger().info(
            f"Relay sender started. Remote={self.remote_host}:{self.remote_port}"
        )

    def _connect_locked(self):
        try:
            sock = socket.create_connection(
                (self.remote_host, self.remote_port), timeout=3.0
            )
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
            self.get_logger().info("Connected to relay receiver")
        except OSError as exc:
            self._sock = None
            self.get_logger().warn(f"Relay receiver unavailable: {exc}")

    def _close_locked(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _ensure_connection(self):
        with self._sock_lock:
            if self._sock is None:
                self._connect_locked()

    def _on_trajectory(self, msg: JointTrajectory):
        payload = message_to_payload(msg)
        packet = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")

        with self._sock_lock:
            if self._sock is None:
                self._connect_locked()
                if self._sock is None:
                    return
            try:
                self._sock.sendall(packet)
            except OSError:
                self._close_locked()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelaySender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
