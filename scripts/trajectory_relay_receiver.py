#!/usr/bin/env python3
import json
import os
import queue
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def payload_to_message(payload: dict) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = [str(name) for name in payload.get("joint_names", [])]

    for point_payload in payload.get("points", []):
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in point_payload.get("positions", [])]
        point.velocities = [float(v) for v in point_payload.get("velocities", [])]
        point.accelerations = [float(v) for v in point_payload.get("accelerations", [])]
        point.effort = [float(v) for v in point_payload.get("effort", [])]

        tfs = point_payload.get("time_from_start", {})
        point.time_from_start.sec = int(tfs.get("sec", 0))
        point.time_from_start.nanosec = int(tfs.get("nanosec", 0))

        msg.points.append(point)

    return msg


class TrajectoryRelayReceiver(Node):
    def __init__(self):
        super().__init__("trajectory_relay_receiver")

        self.listen_host = os.getenv("TRAJECTORY_RELAY_LISTEN_HOST", "0.0.0.0")
        self.listen_port = int(os.getenv("TRAJECTORY_RELAY_LISTEN_PORT", "15555"))

        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory", 10)

        self._queue = queue.Queue()
        self.create_timer(0.05, self._drain_queue)

        thread = threading.Thread(target=self._server_loop, daemon=True)
        thread.start()

        self.get_logger().info(
            f"Relay receiver listening on {self.listen_host}:{self.listen_port}"
        )

    def _server_loop(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server.bind((self.listen_host, self.listen_port))
                    server.listen(1)

                    while True:
                        conn, addr = server.accept()
                        self.get_logger().info(
                            f"Relay sender connected from {addr[0]}:{addr[1]}"
                        )
                        with conn:
                            file_obj = conn.makefile("r", encoding="utf-8")
                            for line in file_obj:
                                line = line.strip()
                                if not line:
                                    continue
                                try:
                                    self._queue.put(json.loads(line))
                                except json.JSONDecodeError:
                                    continue
            except OSError as exc:
                self.get_logger().warn(f"Relay receiver socket error: {exc}")
                time.sleep(1.0)

    def _drain_queue(self):
        while True:
            try:
                payload = self._queue.get_nowait()
            except queue.Empty:
                break

            try:
                self.publisher.publish(payload_to_message(payload))
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish relayed trajectory: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelayReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
