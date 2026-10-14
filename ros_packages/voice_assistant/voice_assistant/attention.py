"""Camera and microphone wiring for attention. Decisions are pure."""

from __future__ import annotations

import logging
import time

from pib_hermes_config.attention import (
    AttentionPlan,
    SceneBuffer,
    assess,
    body_from_detection,
    greeting_line,
    remember_greeting,
)

logger = logging.getLogger(__name__)

GREETING_CHECK_SECONDS = 1.0
PERSON_TOPIC = "detections/yolov6n_coco_640x640"
HEAD_POSE_TOPIC = "detections/head_pose_estimation_crop"


class AttentionHandle:
    """Applies a plan: the head moves before the caller speaks or calls the model."""

    def __init__(self, buffer: SceneBuffer, command) -> None:
        self.buffer = buffer
        self._command = command

    def before_answer(self, now: float | None = None) -> AttentionPlan:
        """Turn toward the speaker, then return the preface for this answer."""
        moment = time.monotonic() if now is None else float(now)
        plan = self._plan(moment)
        self._command_plan(plan)
        if plan.person_id:
            self._remember(plan.person_id, moment)
        return plan

    def take_opening_greeting(
        self, language: str | None = None, now: float | None = None
    ) -> str | None:
        """The one greeting for a person who is looking, or nothing."""
        moment = time.monotonic() if now is None else float(now)
        plan = self._plan(moment)
        if not plan.greeting or not plan.person_id or not plan.name:
            return None
        line = greeting_line(plan.name, language)
        self._command_plan(plan)
        if not self._remember(plan.person_id, moment):
            return None
        return line

    def command_toward_speaker(self, now: float | None = None) -> None:
        """Turn as speech starts, before a live answer has audio."""
        moment = time.monotonic() if now is None else float(now)
        self._command_plan(self._plan(moment))

    def _plan(self, now: float) -> AttentionPlan:
        try:
            return assess(self.buffer, now)
        except Exception:
            logger.warning("attention plan failed", exc_info=True)
            return AttentionPlan(now=now)

    def _remember(self, person_id: str, now: float) -> bool:
        try:
            remember_greeting(person_id, now)
        except Exception:
            logger.warning("could not record greeting cooldown", exc_info=True)
            return False
        return True

    def _command_plan(self, plan: AttentionPlan) -> None:
        if plan.head_motor is None or plan.head_position is None:
            return
        try:
            self._command(plan.head_motor, int(plan.head_position))
        except Exception:
            logger.warning("head turn failed", exc_info=True)


def bind_attention(node, *, command_on_speech: bool = False, speech_allowed=None):
    """Subscribe to the sensors that already exist. A missing message type is not fatal."""
    buffer = SceneBuffer()
    command = _noop
    handle = AttentionHandle(buffer, command)
    try:
        command = _ros_command(node)
        handle = AttentionHandle(buffer, command)
        _subscribe(
            node,
            buffer,
            handle if command_on_speech else None,
            speech_allowed,
        )
    except Exception as exc:
        node.get_logger().warning("attention sensors unavailable: %s", exc)
        handle = AttentionHandle(buffer, _noop)
    return handle


def _noop(_motor, _position) -> None:
    return None


def _ros_command(node):
    from datatypes.srv import ApplyJointTrajectory

    client = node.create_client(ApplyJointTrajectory, "apply_joint_trajectory")

    def command(motor, position) -> None:
        ready = getattr(client, "service_is_ready", None)
        if ready is None or not ready():
            return
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        request = ApplyJointTrajectory.Request()
        trajectory = JointTrajectory()
        trajectory.joint_names = [str(motor)]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        trajectory.points = [point]
        request.joint_trajectory = trajectory
        client.call_async(request)
        node.get_logger().info(
            "turning %s to %s before answering", str(motor), int(position)
        )

    return command


def _subscribe(node, buffer: SceneBuffer, speech_handle, speech_allowed) -> None:
    from datatypes.msg import DetectionArray
    from std_msgs.msg import Bool, Float32MultiArray, Int32

    def on_doa(msg) -> None:
        buffer.note_doa(int(msg.data))

    def on_face(msg) -> None:
        data = list(getattr(msg, "data", []) or [])
        if len(data) < 2:
            buffer.note_face_center(0.0, 0.0, time.monotonic())
            return
        buffer.note_face_center(float(data[0]), float(data[1]), time.monotonic())

    def on_detections(source):
        def callback(msg) -> None:
            width = int(getattr(msg, "frame_width", 0) or 0)
            height = int(getattr(msg, "frame_height", 0) or 0)
            bodies = []
            for detection in getattr(msg, "detections", []) or []:
                body = body_from_detection(detection, width, height)
                if body is not None:
                    bodies.append(body)
            buffer.note_bodies(
                source,
                bodies,
                time.monotonic(),
                width or None,
                height or None,
            )

        return callback

    def on_speech(msg) -> None:
        if speech_handle is None:
            return
        if not buffer.note_speech(bool(msg.data)):
            return
        if speech_allowed is not None and not speech_allowed():
            return
        speech_handle.command_toward_speaker()

    node.create_subscription(Int32, "/doa_angle", on_doa, 10)
    node.create_subscription(Float32MultiArray, "face_center", on_face, 10)
    node.create_subscription(Bool, "/speech_detected", on_speech, 10)
    node.create_subscription(DetectionArray, PERSON_TOPIC, on_detections("person"), 10)
    node.create_subscription(
        DetectionArray, HEAD_POSE_TOPIC, on_detections("head_pose"), 10
    )
