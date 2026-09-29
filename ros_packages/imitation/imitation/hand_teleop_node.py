"""Typed hand-tracking consumer with opt-in, rate-limited arm teleoperation."""

from __future__ import annotations

import math
import os
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from datatypes.msg import DetectionArray
from datatypes.srv import ApplyJointTrajectory, StartModel, StopModel
from geometry_msgs.msg import PoseStamped
from pib_sdk.kinematics import ArmKinematics
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from .landmark_mapping import PalmPose, map_to_robot_target, palm_pose_from_landmarks
from .trajectory import TeleopController, TimedTrajectory

MODEL_ID = "hand_tracking"
DETECTION_TOPIC = "/detections/hand_tracking"
HAND_POSE_TOPIC = "/imitation/hand_pose"
TARGET_POSE_TOPIC = "/imitation/target_pose"


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1_000_000_000)))


def trajectory_request(trajectory: TimedTrajectory) -> ApplyJointTrajectory.Request:
    """Convert the pure full-joint representation to the existing ROS service."""
    request = ApplyJointTrajectory.Request()
    request.joint_trajectory.joint_names = list(trajectory.joint_names)
    for waypoint in trajectory.waypoints:
        point = JointTrajectoryPoint()
        point.positions = list(waypoint.positions)
        point.time_from_start = _duration(waypoint.time_from_start)
        request.joint_trajectory.points.append(point)
    return request


class HandTeleopNode(Node):
    """Own hand inference and expose poses, with motor commands disabled by default."""

    def __init__(self) -> None:
        super().__init__("hand_teleop")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("arm", "right")
        self.declare_parameter("max_update_hz", 10.0)
        self.declare_parameter("trajectory_duration_sec", 0.25)
        self.declare_parameter("fallback_depth_mm", 500.0)
        self.declare_parameter("palm_width_mm", 80.0)
        self.declare_parameter("min_detection_score", 0.5)
        self.declare_parameter("target_origin_mm", [180.0, -180.0, 220.0])
        self.declare_parameter("reference_depth_mm", 500.0)
        self.declare_parameter("position_gain", 1.0)
        self.declare_parameter("use_orientation", False)

        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        arm = str(self.get_parameter("arm").value).lower()
        if arm not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'")
        update_hz = float(self.get_parameter("max_update_hz").value)
        if not math.isfinite(update_hz) or update_hz <= 0.0:
            raise ValueError("max_update_hz must be finite and positive")

        self.owner = f"imitation-hand-teleop-{os.getpid()}"
        self._latest_detection: DetectionArray | None = None
        self._model_acquired = False
        self._model_request_in_flight = False
        self._shutting_down = False

        self.start_model_client = self.create_client(StartModel, "/start_model")
        self.stop_model_client = self.create_client(StopModel, "/stop_model")
        self.trajectory_client = self.create_client(
            ApplyJointTrajectory, "/apply_joint_trajectory"
        )
        self.controller = TeleopController(
            ArmKinematics(arm),
            self.trajectory_client,
            enable_motion=self.enable_motion,
            duration_sec=float(self.get_parameter("trajectory_duration_sec").value),
            use_orientation=bool(self.get_parameter("use_orientation").value),
        )

        self.hand_pose_publisher = self.create_publisher(
            PoseStamped, HAND_POSE_TOPIC, 10
        )
        self.target_pose_publisher = self.create_publisher(
            PoseStamped, TARGET_POSE_TOPIC, 10
        )
        self.create_subscription(
            DetectionArray, DETECTION_TOPIC, self._on_detection, 10
        )
        self.create_timer(1.0 / update_hz, self._process_latest)
        self.create_timer(1.0, self._ensure_model_acquired)

        state = "ENABLED" if self.enable_motion else "disabled (dry-run)"
        self.get_logger().warn(f"hand teleoperation motion is {state}")
        self._ensure_model_acquired()

    def _on_detection(self, message: DetectionArray) -> None:
        """Coalesce camera callbacks; only the newest frame is processed."""
        self._latest_detection = message

    def _ensure_model_acquired(self) -> None:
        if (
            self._shutting_down
            or self._model_acquired
            or self._model_request_in_flight
            or not self.start_model_client.service_is_ready()
        ):
            return
        request = StartModel.Request()
        request.model_id = MODEL_ID
        request.shaves = 0
        request.owner = self.owner
        self._model_request_in_flight = True
        future = self.start_model_client.call_async(request)
        future.add_done_callback(self._on_model_started)

    def _on_model_started(self, future: Any) -> None:
        self._model_request_in_flight = False
        try:
            response = future.result()
            self._model_acquired = bool(response and response.success)
            if not self._model_acquired:
                message = getattr(response, "message", "no response")
                self.get_logger().error(f"could not acquire hand_tracking: {message}")
        except Exception as exc:
            self.get_logger().error(f"start_model failed: {exc}")

    def _process_latest(self) -> None:
        message = self._latest_detection
        self._latest_detection = None
        if message is None or not message.detections:
            return
        started = time.perf_counter()
        try:
            minimum_score = float(self.get_parameter("min_detection_score").value)
            candidates = [
                item
                for item in message.detections
                if math.isfinite(float(item.score))
                and float(item.score) >= minimum_score
            ]
            if not candidates:
                return
            detection = max(candidates, key=lambda item: float(item.score))
            hand_pose = palm_pose_from_landmarks(
                detection.keypoint_names,
                detection.keypoint_x,
                detection.keypoint_y,
                detection.keypoint_z,
                frame_width=int(message.frame_width),
                frame_height=int(message.frame_height),
                fallback_depth_mm=float(self.get_parameter("fallback_depth_mm").value),
                palm_width_mm=float(self.get_parameter("palm_width_mm").value),
            )
            target_pose = map_to_robot_target(
                hand_pose,
                origin_mm=self.get_parameter("target_origin_mm").value,
                reference_depth_mm=float(
                    self.get_parameter("reference_depth_mm").value
                ),
                position_gain=float(self.get_parameter("position_gain").value),
            )
            self.hand_pose_publisher.publish(
                self._pose_message(hand_pose, message.header, "camera_optical_frame")
            )
            self.target_pose_publisher.publish(
                self._pose_message(target_pose, message.header, "robot_base")
            )
            if not self.controller.request_in_flight:
                command = self.controller.prepare(
                    (
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z,
                    ),
                    target_pose.quaternion_xyzw,
                )
                if command is not None:
                    future = self.controller.submit(trajectory_request(command))
                    if future is not None:
                        future.add_done_callback(self._on_trajectory_complete)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            latency_log = f"processing_latency_ms={elapsed_ms:.3f}"
            if stamp_ns > 0:
                source_latency_ms = (
                    self.get_clock().now().nanoseconds - stamp_ns
                ) / 1e6
                latency_log += f" source_latency_ms={source_latency_ms:.3f}"
            self.get_logger().debug(latency_log)
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warn(f"rejected hand target: {exc}")
        except Exception as exc:
            self.get_logger().error(f"hand target processing failed safely: {exc}")

    def _pose_message(
        self, pose: PalmPose, source_header: Any, frame: str
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = source_header.stamp
        message.header.frame_id = source_header.frame_id or frame
        if frame == "robot_base":
            message.header.frame_id = frame
        # geometry_msgs uses SI units; mapping and the SDK intentionally use mm.
        message.pose.position.x = pose.position.x / 1000.0
        message.pose.position.y = pose.position.y / 1000.0
        message.pose.position.z = pose.position.z / 1000.0
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = pose.quaternion_xyzw
        return message

    def _on_trajectory_complete(self, future: Any) -> None:
        try:
            response = future.result()
            if response is None or not response.successful:
                self.get_logger().error("arm trajectory service rejected the command")
        except Exception as exc:
            self.get_logger().error(f"arm trajectory service failed: {exc}")

    def release_model(self) -> Any | None:
        """Release this process's model reference without touching other owners."""
        if not self._model_acquired or not self.stop_model_client.service_is_ready():
            return None
        request = StopModel.Request()
        request.model_id = MODEL_ID
        request.owner = self.owner
        self._model_acquired = False
        return self.stop_model_client.call_async(request)

    def destroy_node(self) -> bool:
        self._shutting_down = True
        self.release_model()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HandTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node._shutting_down = True
        release_future = node.release_model()
        if release_future is not None:
            rclpy.spin_until_future_complete(node, release_future, timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
