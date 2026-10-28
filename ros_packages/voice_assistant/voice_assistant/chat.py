import json
import os
import re
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from queue import Empty, Queue
from threading import Lock
from typing import Any, Optional

import logging

import rclpy
import datetime
from datatypes.action import Chat
from datatypes.msg import ChatMessage
from datatypes.srv import GetCameraImage, VisionPrompt

# NEW: service for AudioLoop → ChatNode bridge (keeps AudioLoop thin)
from datatypes.srv import CreateOrUpdateChatMessage

from pib_api_client import voice_assistant_client
from public_api_client.public_voice_client import PublicApiChatMessage
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service

logger = logging.getLogger(__name__)
from std_msgs.msg import String

from pib_hermes_config.local_model import (
    API_NAME as LOCAL_MODEL_API_NAME,
    iter_reply,
    openai_base_url,
)
from pib_hermes_config import provider_for_profile
from pib_hermes_config.turn_taking import next_speech_delta, speech_result_tail
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    direct_system_prompt,
    personality_system_prompt,
    turn_channel,
)
from pib_hermes_config.retired_direct_text import (
    RETIRED_DIRECT_TEXT_SENTENCE,
    legacy_direct_text_selected,
)
from provider_registry import (
    PIB_CLOUD_API_NAME,
    provider_name_for,
)
from public_api_client import hermes_agent_client, public_voice_client
from voice_assistant import direct_tool_loop, pib_cloud_stream, robot_context
from voice_assistant.direct_session import DirectSession
from voice_assistant.degraded_chat import (
    MODE_UNLOCKED,
    fetch_operating_mode,
    refusal_sentence,
)
from voice_assistant.memory_consolidation import schedule_memory_consolidation
from voice_assistant.chat_turn import (
    STATE_CANCELLED,
    STATE_COMPLETED,
    STATE_RUNNING,
    turn_state_event,
)

# How often the hermes turn is interrupted to notice a cancel request.
HERMES_CANCEL_POLL_SECONDS = 0.2

# Slack on top of the hermes turn's own subprocess timeout, so a wedged worker
# can never hold a goal open forever.
HERMES_WAIT_GRACE_SECONDS = 15

# Upper bound for the startup liveness probe. Node construction blocks on it, so
# it stays short; a failed probe only downgrades hermes personalities to the
# fallback reply and must never keep the node from starting.
HERMES_PROBE_TIMEOUT_SECONDS = 5


#: A camera frame is far too large for a chat attachment, so an image a tool
#: captured is scaled down once, here, before it leaves the robot.
CHAT_IMAGE_MAX_EDGE = 1024
CHAT_IMAGE_QUALITY = 80


def prepare_tool_image(
    image_base64: str,
    mime_type: str = "image/jpeg",
    *,
    max_edge: int = CHAT_IMAGE_MAX_EDGE,
    quality: int = CHAT_IMAGE_QUALITY,
) -> Optional[dict]:
    """Scale one captured picture down and re-encode it as JPEG.

    Answers ``{imageBase64, mimeType, width, height}``, or ``None`` when the
    payload cannot be read: a tool that returned something undecodable must not
    take the turn down, the answer is simply stored without a picture.

    Pillow is imported here, not at module level, so a runtime image without it
    still speaks - it just cannot attach pictures.
    """
    try:
        import base64 as _base64
        import io as _io

        from PIL import Image
    except Exception:  # noqa: BLE001 - the voice is more important than pictures
        return None
    try:
        raw = _base64.b64decode((image_base64 or "").strip(), validate=False)
        with Image.open(_io.BytesIO(raw)) as image:
            image = image.convert("RGB")
            if max_edge > 0 and max(image.size) > max_edge:
                scale = max_edge / float(max(image.size))
                size = (
                    max(1, int(round(image.width * scale))),
                    max(1, int(round(image.height * scale))),
                )
                image = image.resize(size, Image.LANCZOS)
            buffer = _io.BytesIO()
            image.save(buffer, format="JPEG", quality=quality)
            encoded = _base64.b64encode(buffer.getvalue()).decode("ascii")
            return {
                "imageBase64": encoded,
                "mimeType": "image/jpeg",
                "width": image.width,
                "height": image.height,
            }
    except Exception as exc:  # noqa: BLE001 - a broken picture is not a broken turn
        logger.warning(f"captured image was not prepared: {exc}")
        return None


def _model_provider_name(api_name: Optional[str]) -> Optional[str]:
    """The registry provider name for a model's api_name (PR-1930b).

    The on-device model has no catalogue line, so it is named directly; every
    other model resolves to its provider through the catalogue.
    """
    if api_name == LOCAL_MODEL_API_NAME:
        return "Local"
    if not api_name:
        return None
    return provider_name_for(api_name)


def _model_needs_provider_key(api_name: Optional[str]) -> bool:
    """True when a turn on this model must have an unlocked provider key.

    False for the on-device model: it runs locally and needs no key, so the
    locked-store refusal must not fire for it.
    """
    mapping = provider_for_profile(_model_provider_name(api_name))
    return bool(mapping.env_vars)


def _model_base_url(api_name: Optional[str]) -> Optional[str]:
    """The OpenAI-compatible base URL a custom route needs, or None.

    The on-device model points at the Ollama endpoint; pib.Cloud points at the
    configured pib.Cloud endpoint; the backend writes the same URL into the
    profile at provision time.
    """
    if api_name == LOCAL_MODEL_API_NAME:
        return openai_base_url()
    if api_name == PIB_CLOUD_API_NAME:
        return pib_cloud_stream.pib_cloud_base_url()
    return None


class ChatNode(Node):
    """
    Central chat node.

    Responsibilities:
    - Exposes a ROS 2 Action "chat" for request/response (token streaming).
      Smart and Direct both use this action, the chat_messages topic and the
      same chat store; streaming republishes one message_id.
    - Publishes datatypes/ChatMessage on "chat_messages" so UIs/loggers can subscribe.
    - Talks to PIB API (voice_assistant_client) to persist chat messages.
    - (NEW) Exposes a ROS 2 Service "create_or_update_chat_message" so external nodes
      (e.g., the Gemini audio loop) can CREATE/UPDATE a message while it streams text,
      without re-implementing any persistence/publish logic.
    """

    def __init__(self):
        super().__init__("chat")

        # Token for public API (injected via std_msgs/String topic "public_api_token")
        self.token: Optional[str] = None

        # PIB message bookkeeping for the Action path (create_chat_message):
        self.last_pib_message_id: Optional[str] = None
        self.message_content: Optional[str] = None

        # How many previous messages to include in history for public API requests
        self.history_length: int = 10

        # Action server for communicating with LLM via public-api.
        # Client sends a Chat.Goal {chat_id, text, generate_code}
        # We stream feedback (sentences/code) and return the final chunk as result.
        self.chat_server = ActionServer(
            self,
            Chat,
            "chat",
            execute_callback=self.chat,
            cancel_callback=(lambda _: CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup(),
        )

        # Publisher for ChatMessages (ROS topic that UIs consume)
        self.chat_message_publisher: Publisher = self.create_publisher(
            ChatMessage, "chat_messages", 10
        )

        # Camera image service client (optional context if model supports images)
        self.get_camera_image_client = self.create_client(
            GetCameraImage, "get_camera_image"
        )

        # Subscription for public API token (hot-swapped at runtime)
        self.get_token_subscription = self.create_subscription(
            String, "public_api_token", self.get_public_api_token_listener, 10
        )

        # Locks for shared clients (defensive: public voice client & PIB client)
        self.public_voice_client_lock = Lock()
        self.voice_assistant_client_lock = Lock()
        self._turn_lock = Lock()

        # NEW: Lightweight service for external streamers to create/update + publish
        # a ChatMessage without duplicating persistence logic.
        self._cu_srv: Service = self.create_service(
            CreateOrUpdateChatMessage,
            "create_or_update_chat_message",
            self._handle_create_or_update_chat_message,
            callback_group=ReentrantCallbackGroup(),
        )

        self.vision_prompt_service: Service = self.create_service(
            VisionPrompt,
            "vision_prompt",
            self._handle_vision_prompt,
            callback_group=ReentrantCallbackGroup(),
        )

        # Hermes turns shell out to a CLI that blocks for as long as the LLM
        # takes. One pool for the whole node: rclpy's executor threads stay free
        # for cancel requests and concurrent goals, and no request creates
        # threads of its own.
        self._hermes_executor = ThreadPoolExecutor(
            max_workers=4, thread_name_prefix="hermes-turn"
        )

        self._preflight_hermes_binary()
        self._ensure_hermes_daemon()
        schedule_memory_consolidation(self)
        from voice_assistant.attention import bind_attention

        # Head turn and per-person preface. Sensors are optional: a missing
        # camera message must not stop the node from answering.
        self._attention = bind_attention(self)
        self._reset_turn_meta()
        self._install_turn_trace_emitter()

        self.get_logger().info("Now running CHAT")

    def _install_turn_trace_emitter(self) -> None:
        """Publish pib.Cloud stage events. A failure here must not stop the node."""
        try:
            from voice_assistant.turn_latency import TURN_TRACE_TOPIC

            publisher = self.create_publisher(String, TURN_TRACE_TOPIC, 10)
            self._turn_trace_publisher = publisher

            def emit(raw: str) -> None:
                # The same events a UI consumes also describe this turn in the
                # stored answer (PC-82), so record them before publishing.
                self._record_turn_trace(raw)
                message = String()
                message.data = raw
                publisher.publish(message)

            pib_cloud_stream.set_trace_emitter(emit)
        except Exception as exc:
            self.get_logger().warning(f"turn trace emitter was not installed: {exc}")

    def _turn_trace_chat_id(self) -> str:
        chat_id = getattr(self, "_turn_chat_id", None)
        return chat_id if isinstance(chat_id, str) else ""

    def _record_tool_started(self, record: dict) -> None:
        """Publish that a tool invocation has begun (no argument values)."""
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event, now_ms

            name = str(record.get("name") or "tool")
            call_id = record.get("callId") or record.get("call_id") or ""
            message = String()
            message.data = format_event(
                "tool_call_started",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail={
                    "call_id": call_id,
                    "arg_names": record.get("argNames") or [],
                },
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"tool start trace was not published: {exc}")

    def _record_tool_finished(self, record: dict) -> None:
        """Keep one executed tool of this turn and publish finished trace events.

        The record carries the tool name, its outcome, its duration and the names
        of its arguments - never the values (PC-84).
        """
        calls = getattr(self, "_turn_tool_calls", None)
        if not isinstance(calls, list):
            calls = self._turn_tool_calls = []
        calls.append(record)
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event, now_ms

            name = str(record.get("name") or "tool")
            call_id = record.get("callId") or record.get("call_id") or ""
            detail = {
                "call_id": call_id,
                "ok": bool(record.get("ok", True)),
                "duration_ms": record.get("durationMs"),
                "arg_names": record.get("argNames") or [],
                "error": record.get("error"),
            }
            message = String()
            message.data = format_event(
                "tool_call_finished",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail=detail,
            )
            publisher.publish(message)
            legacy = String()
            legacy.data = format_event(
                "tool_call",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail=detail,
            )
            publisher.publish(legacy)
        except Exception as exc:
            self.get_logger().warning(f"tool call trace was not published: {exc}")

    def _record_tool_call(self, record: dict) -> None:
        """Hermes and other paths that only report after execution."""
        self._record_tool_finished(record)

    def _record_tool_image(self, record: dict) -> None:
        """Keep one picture a tool captured this turn, scaled down.

        Only the camera tool produces one, and only the direct tool loop reports
        it: the Hermes daemon runs its tools itself, so an image captured there
        never reaches this runtime.
        """
        prepared = prepare_tool_image(
            str(record.get("imageBase64") or ""),
            str(record.get("mimeType") or "image/jpeg"),
        )
        if prepared is None:
            return
        images = getattr(self, "_turn_images", None)
        if not isinstance(images, list):
            images = self._turn_images = []
        tool = record.get("tool")
        prepared["tool"] = tool if isinstance(tool, str) else ""
        images.append(prepared)

    def _reset_turn_meta(self) -> None:
        """A new turn forgets the previous turn's facts."""
        self._turn_stages = {}
        self._turn_started_ms = None
        self._turn_fallback_reason = None
        self._turn_stream_end = None
        self._turn_path_token = ""
        self._turn_model = ""
        self._turn_tool_calls = []
        self._turn_images = []
        self._turn_image_references = []
        self._turn_chat_id = ""
        self._turn_assistant_message_id = ""
        self._turn_trace_turn_id = ""
        self._turn_trace_cancelled = False
        self._turn_direct_tool_rounds = None

    def _ensure_turn_trace_anchor(self, t0: float) -> None:
        """Typed turns anchor at service recv when capture never sent end_of_speech."""
        if getattr(self, "_turn_started_ms", None) is None:
            self._turn_started_ms = float(t0) * 1000.0
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        if "ros_service_recv" not in stages:
            stages["ros_service_recv"] = 0.0

    def _record_local_turn_stage(self, stage: str, t0: float) -> None:
        """Record a stage the chat node observes locally (PR-2072)."""
        self._ensure_turn_trace_anchor(t0)
        started = getattr(self, "_turn_started_ms", None)
        if started is None:
            return
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        if stage in stages:
            return
        stages[stage] = round(time.monotonic() * 1000.0 - started, 1)

    def _freeze_turn_done_stage(self) -> None:
        if "turn_end" in getattr(self, "_turn_stages", {}) or "stream_done" in getattr(
            self, "_turn_stages", {}
        ):
            return
        started = getattr(self, "_turn_started_ms", None)
        if started is None:
            return
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        stages["turn_end"] = round(time.monotonic() * 1000.0 - started, 1)

    def _emit_turn_state(
        self, turn_id: str, state: str, chat_id: str, *, reason: str = ""
    ) -> None:
        """Publish running / cancelled / completed for one explicit turn id."""
        if not turn_id:
            return
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            message = String()
            message.data = turn_state_event(
                turn_id, state, chat_id=chat_id, reason=reason
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"turn state trace was not published: {exc}")

    def _discard_assistant_reply(self, chat_id: str) -> None:
        """Drop a partial assistant message after cancel (PR-2069)."""
        message_id = getattr(self, "_turn_assistant_message_id", None)
        if not message_id:
            return
        self._turn_assistant_message_id = ""
        try:
            with self.voice_assistant_client_lock:
                voice_assistant_client.delete_chat_message(chat_id, message_id)
        except Exception as exc:
            self.get_logger().warning(
                f"cancelled assistant message was not deleted chat={chat_id}: {exc}"
            )
        if getattr(self, "last_pib_message_id", None) == message_id:
            self.last_pib_message_id = None
            self.message_content = None

    def _finish_cancelled_turn(self, chat_id: str, goal_handle) -> None:
        """Stop the action, remove partial assistant text, and trace cancelled."""
        self._turn_trace_cancelled = True
        self._freeze_turn_done_stage()
        self._discard_assistant_reply(chat_id)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        turn_id = getattr(self, "_turn_trace_turn_id", "") or ""
        if turn_id:
            self._emit_turn_state(turn_id, STATE_CANCELLED, chat_id)

    def _record_turn_trace(self, raw: str) -> None:
        """Keep stage timings and the fallback reason of the running turn."""
        try:
            event = json.loads(raw)
        except (TypeError, ValueError):
            return
        stage = event.get("stage")
        ts = event.get("ts_ms")
        if stage == "end_of_speech":
            self._turn_stages = {}
            self._turn_started_ms = float(ts) if isinstance(ts, (int, float)) else None
        elif stage == "stream_done":
            self._turn_stream_end = "done"
        reason = (event.get("detail") or {}).get("fallback_reason")
        if isinstance(reason, str) and reason:
            self._turn_fallback_reason = reason
        started = getattr(self, "_turn_started_ms", None)
        if started is not None and isinstance(ts, (int, float)):
            stages = getattr(self, "_turn_stages", None)
            if not isinstance(stages, dict):
                stages = self._turn_stages = {}
            stages[stage] = round(float(ts) - started, 1)

    def _turn_meta(self) -> dict | None:
        """Facts of this turn, stored with the answer (PC-82).

        Empty for a turn without a transport announcement - a user message or a
        turn that never reached a transport must not invent metadata.
        """
        meta: dict = {}
        source = getattr(self, "_turn_path_token", "")
        model = getattr(self, "_turn_model", "")
        stages = getattr(self, "_turn_stages", None)
        reason = getattr(self, "_turn_fallback_reason", None)
        stream_end = getattr(self, "_turn_stream_end", None)
        if source:
            meta["source"] = source
        if model:
            meta["model"] = model
        if isinstance(stages, dict) and stages:
            meta["stagesMs"] = dict(stages)
        if reason:
            meta["fallbackReason"] = reason
        if stream_end:
            meta["streamEnd"] = stream_end
        tool_calls = getattr(self, "_turn_tool_calls", None)
        if isinstance(tool_calls, list) and tool_calls:
            meta["toolCalls"] = list(tool_calls)
        images = getattr(self, "_turn_image_references", None)
        if isinstance(images, list) and images:
            meta["images"] = list(images)
        if isinstance(stages, dict) and stages:
            from voice_assistant.turn_latency import message_turn_trace

            trace = message_turn_trace(
                stages,
                cancelled=bool(getattr(self, "_turn_trace_cancelled", False)),
                tool_rounds=getattr(self, "_turn_direct_tool_rounds", None),
                tool_calls=tool_calls if isinstance(tool_calls, list) else None,
            )
            if trace:
                meta["turnTrace"] = trace
        return meta or None

    def _finalize_message_meta(self, chat_id: str) -> None:
        """Write the completed turn facts onto the answer that was just stored.

        Pictures a tool captured are uploaded here rather than before the answer
        exists: an image is attached to a message id, and the id only exists once
        the answer was stored. What the message carries is the reference the API
        answers, never the bytes (PC-119).
        """
        try:
            self._freeze_turn_done_stage()
            self._attach_turn_images(chat_id)
            meta = self._turn_meta()
            message_id = getattr(self, "last_pib_message_id", None)
            if not meta or not message_id:
                return
            with self.voice_assistant_client_lock:
                successful, _ = voice_assistant_client.update_chat_message(
                    chat_id,
                    getattr(self, "message_content", ""),
                    False,
                    message_id,
                    meta=meta,
                )
            if not successful:
                self.get_logger().warning(
                    f"turn metadata was not stored chat={chat_id}"
                )
        except Exception as exc:
            self.get_logger().warning(f"turn metadata failed: {exc}")
        finally:
            # The next turn starts clean; a recorder end_of_speech sets its anchor.
            self._reset_turn_meta()

    def _attach_turn_images(self, chat_id: str) -> None:
        """Upload the pictures of this turn and add their references to the meta."""
        images = getattr(self, "_turn_images", None)
        message_id = getattr(self, "last_pib_message_id", None)
        if not isinstance(images, list) or not images or not message_id:
            return
        references = []
        for image in images:
            try:
                with self.voice_assistant_client_lock:
                    reference = voice_assistant_client.add_chat_message_image(
                        chat_id, message_id, image["imageBase64"]
                    )
            except (
                Exception
            ) as exc:  # noqa: BLE001 - the answer stands without a picture
                self.get_logger().warning(f"captured image was not stored: {exc}")
                continue
            if not reference:
                self.get_logger().warning(
                    f"captured image was refused chat={chat_id} tool={image.get('tool')}"
                )
                continue
            references.append(reference)
        if references:
            self._turn_image_references = references

    #: Machine token per transport for the turn-path trace event. A UI reads this
    #: to say which runtime answered; the human words above it stay for the log.
    TURN_PATH_TOKENS = {
        "Hermes": "hermes",
        "pib.Cloud": "pib-cloud",
        "the on-device model": "local",
        "the direct tool loop": "direct-tool-loop",
    }

    def _emit_turn_path(self, *, chat_id: str, path: str, model: str | None) -> None:
        """Announce the transport and model of this turn on the trace topic.

        The trace already carries per-stage timings from the recorder, the player
        and the cloud client. None of them names the model that answered, so a UI
        could not tell pib.Cloud from the on-device fallback. This event closes
        that gap; a failure here must not stop the turn.
        """
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event

            self._turn_path_token = self.TURN_PATH_TOKENS.get(path, path)
            self._turn_model = model or ""
            detail = {"model": model} if model else None
            message = String()
            message.data = format_event(
                "turn_path",
                source=self.TURN_PATH_TOKENS.get(path, path),
                chat_id=chat_id,
                detail=detail,
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"turn path trace was not published: {exc}")

    def _attention_plan(self):
        """The plan for this answer. Missing sensors leave the turn unchanged."""
        from pib_hermes_config.attention import AttentionPlan

        attention = getattr(self, "_attention", None)
        if attention is None:
            return AttentionPlan()
        return attention.before_answer()

    def _key_store_mode(self) -> str:
        """Operating mode of the key store. Unreadable means degraded."""
        return fetch_operating_mode()

    def _direct_identity_fingerprint(self, personality: Any) -> str:
        """Changes only when the personality's own text changes (PR-2075 review)."""
        description = str(getattr(personality, "description", "") or "")
        name = str(getattr(personality, "name", "") or "")
        soul_path = str(getattr(personality, "soul_path", "") or "")
        soul_stamp = ""
        if soul_path:
            try:
                soul_stamp = str(os.stat(soul_path).st_mtime_ns)
            except OSError:
                soul_stamp = ""
        return f"{name}\x00{description}\x00{soul_path}\x00{soul_stamp}"

    def _direct_session(self, personality: Any) -> DirectSession:
        """Cached Direct transport state for one personality."""
        personality_id = str(getattr(personality, "personality_id", None) or "")
        sessions = getattr(self, "_direct_sessions", None)
        if sessions is None:
            sessions = {}
            self._direct_sessions = sessions
        session = sessions.get(personality_id)
        if session is None:
            session = DirectSession(personality_id)
            sessions[personality_id] = session
        return session

    def _finish_retired_direct_text(self, goal_handle, chat_id: str, api_name):
        """Succeed the turn with the retired-path sentence. No network call.

        The user line is already stored. This appends the assistant sentence
        through the same chat-message path as any other spoken reply. Channel,
        model and credentials are left as they are.
        """
        if goal_handle.is_cancel_requested:
            self._finish_cancelled_turn(chat_id, goal_handle)
            return Chat.Result()
        sentence = RETIRED_DIRECT_TEXT_SENTENCE
        self.create_chat_message(chat_id, sentence, False, False, True)
        goal_handle.succeed()
        result = Chat.Result()
        result.text = sentence
        result.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
        self.get_logger().info(
            f"retired direct text transport chat={chat_id} model={api_name}"
        )
        return result

    def consolidate_memories(self) -> None:
        """Fold old MEMORY.md entries. Recent entries stay verbatim."""
        from voice_assistant.memory_consolidation import run_periodic_consolidation

        try:
            results = run_periodic_consolidation()
        except Exception as exc:
            self.get_logger().error(f"memory consolidation failed: {exc}")
            return
        consolidated = sum(1 for status in results.values() if status == "consolidated")
        if consolidated:
            self.get_logger().info(f"consolidated memory for {consolidated} profile(s)")

    def destroy_node(self):
        # Abandoned hermes workers must not keep the process alive on shutdown.
        self._hermes_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _ensure_hermes_daemon(self) -> bool:
        """Start the warm Hermes daemon in the background if it is not up yet.

        Best-effort: a failed start never blocks ChatNode construction. Turns
        fall back to oneshot subprocess when the daemon is unreachable.
        """
        try:
            from public_api_client import hermes_daemon

            ok = hermes_daemon.ensure_daemon_running()
        except Exception as exc:
            self.get_logger().warning(
                f"hermes daemon could not be started: {exc}; "
                "turns will use oneshot subprocess fallback"
            )
            return False

        if ok:
            self.get_logger().info(
                f"hermes daemon available at {hermes_daemon.daemon_base_url()}"
            )
        else:
            self.get_logger().warning(
                "hermes daemon did not become reachable; "
                "turns will use oneshot subprocess fallback"
            )
        return ok

    def _preflight_hermes_binary(self) -> bool:
        """Report at startup whether the configured Hermes CLI actually runs.

        Without this, a robot whose hermes install is missing or not mounted looks
        healthy while every Smart personality quietly answers with the
        fallback sentence. This only logs.

        This runs the CLI instead of merely stat-ing it. A file check passed on a
        live robot whose CLI died with exit 127 on every turn, because the wrapper
        execs a venv interpreter that was not mounted into the container.
        """
        path = hermes_agent_client.hermes_bin()
        try:
            ok, detail = hermes_agent_client.probe_binary(
                timeout=HERMES_PROBE_TIMEOUT_SECONDS
            )
        except Exception as exc:
            # The probe may never be the reason the chat node fails to come up.
            ok, detail = False, f"probe raised {exc!r}"

        if ok:
            self.get_logger().info(
                f"hermes agent binary available at {path}"
                + (f" ({detail})" if detail else "")
            )
            return True

        self.get_logger().error(
            f"hermes agent preflight failed for '{path}': {detail}. "
            "Smart personalities will fall back to a "
            "canned reply. Check that the hermes CLI is installed for the pib "
            "user, that PIB_HERMES_BIN points at it, and that ~/.hermes, the "
            "wrapper and the uv-managed Python directory are all bind-mounted "
            "into the ros-voice-assistant service."
        )
        return False

    # ---------- common helpers used by Action and Service ----------

    def _publish_chat_message(
        self, chat_id: str, content: str, is_user: bool, message_id: str | None
    ):
        """
        Build and publish a datatypes/ChatMessage with current timestamp.
        Used by both the action path (create_chat_message) and the service path.

        ``message_id`` is None when the line could not be persisted (the API
        refused it). The ROS field is a string, and the generated converter
        aborts the process on None (``PyUnicode_Check`` assertion), so an
        unpersisted line is published with an empty id.
        """
        msg = ChatMessage()
        msg.chat_id = chat_id
        msg.content = content
        msg.is_user = is_user
        msg.message_id = message_id or ""
        msg.timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.chat_message_publisher.publish(msg)

    # ---------- original Action path DB write helper (kept intact) ----------

    def create_chat_message(
        self,
        chat_id: str,
        text: str,
        is_user: bool,
        update_message: bool,
        update_database: bool,
    ) -> None:
        """
        Writes a new chat-message (or updates the last one) to PIB DB,
        and publishes it on the 'chat_messages' topic.

        - When update_message=False → CREATE new message in PIB (records message_id)
        - When update_message=True  → UPDATE existing PIB message_id with concatenated content
        - update_database controls whether we hit PIB on updates or only update local content
        """
        if text == "":
            return

        with self.voice_assistant_client_lock:
            if update_message:
                # UPDATE path
                if update_database:
                    # concatenate locally AND persist to PIB
                    self.message_content = f"{self.message_content} {text}"
                    if self.last_pib_message_id is not None:
                        successful, _ = voice_assistant_client.update_chat_message(
                            chat_id,
                            self.message_content,
                            is_user,
                            self.last_pib_message_id,
                        )
                        if not successful:
                            self.get_logger().error(
                                "unable to update chat message: "
                                f"chat={chat_id} chars={len(text)} "
                                f"message_id={self.last_pib_message_id} user={is_user}"
                            )
                            return
                else:
                    # concatenate locally ONLY
                    self.message_content = f"{self.message_content} {text}"
            else:
                # CREATE path
                successful, chat_message = voice_assistant_client.create_chat_message(
                    chat_id,
                    text,
                    is_user,
                    meta=None if is_user else self._turn_meta(),
                )
                if not successful or chat_message is None:
                    self.get_logger().error(
                        "unable to create chat message: "
                        f"chat={chat_id} chars={len(text)} "
                        f"update={update_message} persist={update_database} user={is_user}"
                    )
                    if is_user:
                        self.get_logger().warning(
                            f"user line was not persisted for chat {chat_id}"
                        )
                        self.last_pib_message_id = None
                        self.message_content = text
                    else:
                        return
                else:
                    self.last_pib_message_id = chat_message.message_id
                    self.message_content = text
                    if not is_user and not update_message:
                        self._turn_assistant_message_id = chat_message.message_id

        # Publish to ROS so UIs/loggers see it immediately.
        self._publish_chat_message(
            chat_id=chat_id,
            content=self.message_content,
            is_user=is_user,
            message_id=self.last_pib_message_id,
        )

    # ---------- NEW: Service handler for AudioLoop streaming (stateless) ----------

    def _handle_create_or_update_chat_message(
        self,
        req: CreateOrUpdateChatMessage.Request,
        resp: CreateOrUpdateChatMessage.Response,
    ) -> CreateOrUpdateChatMessage.Response:
        """
        External, stateless path used by audio_loop.py.

        Contract:
        - AudioLoop sends FULL current text (no delta) and either an empty message_id (CREATE)
          or a non-empty message_id (UPDATE that exact message to the full text).
        - We persist to PIB DB (create/update) and then publish a ChatMessage to ROS.
        - We DO NOT rely on ChatNode's internal last_pib_message_id / message_content here,
          so concurrent clients won't step on each other.
        """
        try:
            chat_id = (req.chat_id or "").strip()
            text = (req.text or "").strip()
            is_user = bool(req.is_user)
            update_db = bool(req.update_database)
            message_id_in = (req.message_id or "").strip()

            if not chat_id or not text:
                resp.successful = False
                resp.message_id = message_id_in
                resp.content = text
                return resp

            # CREATE vs UPDATE in PIB (stateless)
            if message_id_in:
                # UPDATE to EXACT content passed in `text` (no concatenation here)
                if update_db:
                    successful, _ = voice_assistant_client.update_chat_message(
                        chat_id, text, is_user, message_id_in
                    )
                    if not successful:
                        resp.successful = False
                        resp.message_id = message_id_in
                        resp.content = text
                        return resp
                effective_message_id = message_id_in
            else:
                # CREATE a new message
                successful, cm = voice_assistant_client.create_chat_message(
                    chat_id, text, is_user
                )
                if not successful or cm is None:
                    resp.successful = False
                    resp.message_id = ""
                    resp.content = text
                    return resp
                effective_message_id = cm.message_id

            # Publish to topic for subscribers (UIs/loggers)
            self._publish_chat_message(chat_id, text, is_user, effective_message_id)

            # Fill response
            resp.successful = True
            resp.message_id = effective_message_id
            resp.content = text
            return resp

        except Exception as e:
            self.get_logger().error(f"CreateOrUpdateChatMessage failed: {e}")
            resp.successful = False
            resp.message_id = req.message_id
            resp.content = req.text
            return resp

    def _handle_vision_prompt(
        self,
        req: VisionPrompt.Request,
        resp: VisionPrompt.Response,
    ) -> VisionPrompt.Response:
        prompt = (req.prompt or "").strip()

        if not prompt:
            resp.response = "0"
            return resp

        # Blockly vision used the retired text endpoint. It must not call it,
        # and a missing token must not be the reason the caller sees.
        self.get_logger().error(
            "VisionPrompt refused: the tryb text endpoint is retired."
        )
        resp.response = "0"
        return resp

    # ---------- Action server (unchanged) ----------

    def get_public_api_token_listener(self, msg):
        """Receives the token for public_api via ROS topic 'public_api_token'."""
        token = msg.data
        self.token = token

    def _stream_chunks_to_goal(
        self, goal_handle, chat_id: str, tokens, t0: Optional[float] = None
    ) -> tuple[Optional[str], Optional[int], str]:
        """Consume a token iterable, publishing sentence/code chunks as feedback.

        Returns (prev_text, prev_text_type, curr_text) so the caller can build Chat.Result.

        Speech feedback and Chat.Result use a delta-only contract: each payload
        carries only speakable text not yet emitted on the wire (empty final is
        valid). Persisted chat messages stay cumulative. For TTFT, the first
        non-empty token is published immediately before a terminator arrives.
        """
        if t0 is None:
            t0 = time.monotonic()

        # Regex for sentence / code chunking
        sentence_pattern = re.compile(
            r"^(?!<pib-program>)(.*?)(([^\d | ^A-Z][\.|!|\?|:])|<pib-program>)",
            re.DOTALL,
        )
        code_visual_pattern = re.compile(
            r"^<pib-program>(.*?)</pib-program>", re.DOTALL
        )

        # Current and previous text fragments for feedback + persistence
        curr_text: str = ""
        prev_text: Optional[str] = None
        prev_text_type = None
        bool_update_chat_message: bool = False  # controls create vs update
        first_chunk_emitted = False
        first_sentence_flushed = False
        # Full answer as received, kept so a clause can be spoken before the
        # sentence that contains it has finished.
        raw_answer = ""
        published_speech: list[str] = []

        for token in tokens:
            # TTFT fast-path: emit the first generated token immediately, before
            # waiting for a sentence terminator / next-token publish cycle.
            if not first_chunk_emitted and token:
                immediate = token.lstrip() if len(curr_text) == 0 else token
                if immediate:
                    feedback = Chat.Feedback()
                    feedback.text = immediate
                    feedback.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
                    goal_handle.publish_feedback(feedback)
                    first_chunk_emitted = True
                    published_speech.append(immediate)
                    elapsed_ms = (time.monotonic() - t0) * 1000.0
                    self._remember_first_token_latency(chat_id, elapsed_ms)
                    self._record_local_turn_stage("first_token", t0)
                    self.get_logger().info(
                        f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                        f"elapsed_ms={elapsed_ms:.2f}"
                    )

            # Blockly code blocks still publish on the next token; speech does not.
            if (
                prev_text is not None
                and prev_text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL
            ):
                feedback = Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

            # Accumulate token (strip leading spaces if first)
            piece = token if len(curr_text) > 0 else token.lstrip()
            curr_text = curr_text + piece
            raw_answer = raw_answer + piece

            # Strip off complete chunks (code/sentences)
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return prev_text, prev_text_type, curr_text

                # Visual code block
                code_visual_match = code_visual_pattern.search(curr_text)
                if code_visual_match is not None:
                    code_visual = code_visual_match.group(1)
                    prev_text = code_visual
                    prev_text_type = Chat.Goal.TEXT_TYPE_CODE_VISUAL
                    chat_message_text = code_visual_match.group(0)
                    # Written inline instead of as an executor task: an UPDATE
                    # may only run once the preceding CREATE has recorded the
                    # message id it targets, otherwise the concurrent writes
                    # race and the rest of the reply lands on the wrong row.
                    self.create_chat_message(
                        chat_id,
                        chat_message_text,
                        False,
                        bool_update_chat_message,
                        True,
                    )
                    bool_update_chat_message = True
                    curr_text = curr_text[code_visual_match.end() :].rstrip()
                    continue

                # Sentence
                sentence_match = sentence_pattern.search(curr_text)
                if sentence_match is not None:
                    sentence = sentence_match.group(1) + (
                        sentence_match.group(3)
                        if sentence_match.group(3) is not None
                        else ""
                    )
                    prev_text = sentence
                    prev_text_type = Chat.Goal.TEXT_TYPE_SENTENCE
                    if not first_sentence_flushed:
                        first_sentence_flushed = True
                        elapsed_ms = (time.monotonic() - t0) * 1000.0
                        self._record_local_turn_stage("first_sentence_flushed", t0)
                        self.get_logger().info(
                            f"[PERF_TRACE] FIRST_SENTENCE_FLUSHED chat={chat_id} "
                            f"elapsed_ms={elapsed_ms:.2f}"
                        )
                    chat_message_text = sentence
                    self.create_chat_message(
                        chat_id,
                        chat_message_text,
                        False,
                        bool_update_chat_message,
                        True,
                    )
                    bool_update_chat_message = True
                    curr_text = curr_text[
                        sentence_match.end(
                            3 if sentence_match.group(3) is not None else 1
                        ) :
                    ].rstrip()
                    continue

                break

            if self._publish_ready_clauses(
                goal_handle,
                raw_answer,
                published_speech,
                chat_id=chat_id,
                t0=t0 if not first_sentence_flushed else None,
            ):
                first_sentence_flushed = True

        # A reply can end without a sentence terminator. That tail is part of
        # the answer, so it is persisted here instead of being dropped.
        leftover = curr_text.strip()
        if leftover:
            self.create_chat_message(
                chat_id,
                leftover,
                False,
                bool_update_chat_message,
                True,
            )
            if not first_chunk_emitted:
                first_chunk_emitted = True
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                self._remember_first_token_latency(chat_id, elapsed_ms)
                self._record_local_turn_stage("first_token", t0)
                self.get_logger().info(
                    f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                    f"elapsed_ms={elapsed_ms:.2f}"
                )
            if self._publish_ready_clauses(
                goal_handle,
                raw_answer,
                published_speech,
                chat_id=chat_id,
                t0=t0 if not first_sentence_flushed else None,
            ):
                first_sentence_flushed = True
            if (
                prev_text is not None
                and prev_text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL
            ):
                feedback = Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

        if prev_text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL:
            return prev_text, prev_text_type, curr_text

        speech_tail = speech_result_tail(raw_answer, published_speech)
        return None, None, speech_tail

    def _remember_first_token_latency(self, chat_id: str, elapsed_ms: float) -> None:
        """Persist the measurement. A down API must not fail the turn."""
        try:
            voice_assistant_client.record_first_token_latency(chat_id, elapsed_ms)
        except Exception as exc:
            self.get_logger().warning(
                f"first-token latency was not stored for chat {chat_id}: {exc}"
            )

    def _publish_ready_clauses(
        self,
        goal_handle,
        raw_answer: str,
        published_speech: list[str],
        chat_id: str = "",
        t0: Optional[float] = None,
    ) -> bool:
        """Publish the next incremental speech delta so TTS can start early."""
        flushed_any = False
        while True:
            delta = next_speech_delta(raw_answer, published_speech)
            if not delta:
                break
            feedback = Chat.Feedback()
            feedback.text = delta
            feedback.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
            goal_handle.publish_feedback(feedback)
            if not flushed_any and t0 is not None:
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                self._record_local_turn_stage("first_sentence_flushed", t0)
                self.get_logger().info(
                    f"[PERF_TRACE] FIRST_SENTENCE_FLUSHED chat={chat_id} "
                    f"elapsed_ms={elapsed_ms:.2f}"
                )
            flushed_any = True
            published_speech.append(delta)
        return flushed_any

    def _hermes_timeout(self) -> int:
        """Timeout for one hermes turn, read live so ops can tune it via env."""
        return int(
            os.environ.get(
                "PIB_HERMES_TIMEOUT", hermes_agent_client.DEFAULT_TIMEOUT_SECONDS
            )
        )

    def _run_hermes_turn(
        self,
        text: str,
        chat_id: str,
        personality_id: Optional[str],
        description: str,
        timeout: Optional[int] = None,
        goal_handle=None,
        model: Optional[str] = None,
        provider: Optional[str] = None,
        base_url: Optional[str] = None,
        tool_calling: bool = True,
        message_history: Optional[int] = None,
    ) -> str:
        """Run one hermes turn and return speakable text. Never raises.

        Deliberately synchronous and asyncio-free: rclpy drives the action
        server's execute_callback with its own executor, so the calling thread
        has no asyncio event loop and asyncio.get_running_loop() would raise
        RuntimeError('no running event loop').

        Returns "" only when the goal was cancelled while the agent was running;
        every agent failure yields the fallback sentence so the goal can still
        succeed.
        """
        if timeout is None:
            timeout = self._hermes_timeout()

        def _turn() -> str:
            # Smart turns seed the Hermes session from flask chat messages (the
            # record the user saw) before the agent runs; persona still comes
            # from the profile (-p).
            # When the warm daemon is already up, skip filesystem profile
            # re-validation — profiles were provisioned at personality create
            # time / prior cold starts and re-writing SOUL.md adds multi-second
            # latency on the Pi.
            # Always ensure new profile directories receive config.yaml / .env
            if personality_id:
                pdir = hermes_agent_client.profile_dir_for(personality_id)
                cfg_file = os.path.join(pdir, "config.yaml")
                if (
                    not os.path.exists(cfg_file)
                    or not hermes_agent_client.is_warm_daemon_active()
                ):
                    hermes_agent_client.ensure_profile(
                        personality_id,
                        soul_text=description,
                        model=model,
                        provider=provider,
                        base_url=base_url,
                        tool_calling=tool_calling,
                    )
            disabled_toolsets = (
                hermes_agent_client.DEFAULT_DISABLED_TOOLSETS
                if tool_calling
                else hermes_agent_client.disabled_toolsets_for_turn(False)
            )
            enabled_toolsets = (
                hermes_agent_client.DEFAULT_ENABLED_TOOLSETS
                if tool_calling
                else hermes_agent_client.enabled_toolsets_for_turn(False)
            )
            return hermes_agent_client.run_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                toolsets=disabled_toolsets,
                enabled_toolsets=enabled_toolsets,
                timeout=timeout,
                message_history=message_history,
            )

        future = self._hermes_executor.submit(_turn)
        deadline = time.monotonic() + timeout + HERMES_WAIT_GRACE_SECONDS

        while True:
            try:
                return future.result(timeout=HERMES_CANCEL_POLL_SECONDS)
            except FutureTimeoutError:
                pass
            except direct_tool_loop.DirectToolLoopError:
                # The store could not supply the provider key. The goal aborts
                # with that message; the fallback sentence would hide it.
                raise
            except Exception as exc:
                self.get_logger().error(f"hermes agent turn failed: {exc}")
                return hermes_agent_client.FALLBACK_REPLY

            if goal_handle is not None and goal_handle.is_cancel_requested:
                # The worker is left to finish on its own; the shared pool
                # outlives this goal.
                self.get_logger().info(
                    f"hermes turn abandoned, goal cancelled (chat={chat_id})"
                )
                return ""

            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"hermes turn exceeded {timeout}s plus grace "
                    f"(chat={chat_id}); answering with the fallback reply"
                )
                return hermes_agent_client.FALLBACK_REPLY

    def _stream_hermes_turn(
        self,
        text: str,
        chat_id: str,
        personality_id: Optional[str],
        description: str,
        goal_handle=None,
        model: Optional[str] = None,
        provider: Optional[str] = None,
        base_url: Optional[str] = None,
        tool_calling: bool = True,
        message_history: Optional[int] = None,
    ):
        """Yield daemon deltas, retrying through the non-streaming path on error."""
        timeout = self._hermes_timeout()
        events: Queue = Queue()

        def _consume_stream() -> None:
            try:
                if personality_id:
                    pdir = hermes_agent_client.profile_dir_for(personality_id)
                    cfg_file = os.path.join(pdir, "config.yaml")
                    if (
                        not os.path.exists(cfg_file)
                        or not hermes_agent_client.is_warm_daemon_active()
                    ):
                        hermes_agent_client.ensure_profile(
                            personality_id,
                            soul_text=description,
                            model=model,
                            provider=provider,
                            base_url=base_url,
                            tool_calling=tool_calling,
                        )
                disabled_toolsets = (
                    hermes_agent_client.DEFAULT_DISABLED_TOOLSETS
                    if tool_calling
                    else hermes_agent_client.disabled_toolsets_for_turn(False)
                )
                enabled_toolsets = (
                    hermes_agent_client.DEFAULT_ENABLED_TOOLSETS
                    if tool_calling
                    else hermes_agent_client.enabled_toolsets_for_turn(False)
                )
                hermes_tools: list = []
                try:
                    for delta in hermes_agent_client.stream_turn(
                        text=text,
                        chat_id=chat_id,
                        personality_id=personality_id,
                        toolsets=disabled_toolsets,
                        enabled_toolsets=enabled_toolsets,
                        timeout=timeout,
                        tool_sink=hermes_tools,
                        message_history=message_history,
                    ):
                        events.put(("delta", delta))
                    for record in hermes_tools:
                        self._record_tool_call(
                            {
                                "name": str(record.get("name") or "tool"),
                                "ok": bool(record.get("ok", True)),
                                "argNames": [],
                            }
                        )
                    events.put(("done", None))
                except Exception as exc:
                    events.put(("error", exc))
            except Exception as exc:
                events.put(("error", exc))

        self._hermes_executor.submit(_consume_stream)
        deadline = time.monotonic() + timeout + HERMES_WAIT_GRACE_SECONDS
        emitted = ""

        while True:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return
            if time.monotonic() > deadline:
                event = ("error", TimeoutError("Hermes stream exceeded its deadline"))
            else:
                try:
                    event = events.get(timeout=HERMES_CANCEL_POLL_SECONDS)
                except Empty:
                    continue

            kind, value = event
            if kind == "delta":
                emitted += value
                yield value
                continue
            if kind == "done":
                return
            if isinstance(value, direct_tool_loop.DirectToolLoopError):
                raise value

            self.get_logger().warning(
                f"hermes streaming failed (chat={chat_id}): {value}; "
                "retrying without streaming"
            )
            reply = self._run_hermes_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                description=description,
                timeout=timeout,
                goal_handle=goal_handle,
                model=model,
                provider=provider,
                base_url=base_url,
                tool_calling=tool_calling,
                message_history=message_history,
            )
            if emitted and reply.startswith(emitted):
                reply = reply[len(emitted) :]
            if reply:
                yield reply
            return

    def _robot_context(self) -> str:
        """The body description for this turn, empty when the API cannot be asked."""
        try:
            return robot_context.get_robot_context()
        except Exception:
            return ""

    def _iter_local_tokens(self, chat_id: str, content: str, system_prompt: str):
        """Tokens from the on-device model. No provider key is read."""
        history: list[tuple[str, bool]] = []
        try:
            with self.voice_assistant_client_lock:
                successful, chat_messages = voice_assistant_client.get_chat_history(
                    chat_id, self.history_length
                )
            if successful:
                history = [
                    (message.content, message.is_user) for message in chat_messages
                ]
        except Exception as exc:
            self.get_logger().error(
                f"local model history could not be read for chat {chat_id}: {exc}"
            )
        return iter_reply(system_prompt, history, content)

    def _get_turn_lock(self) -> Lock:
        """Return the per-node turn serialization lock (created lazily for test doubles)."""
        lock = getattr(self, "_turn_lock", None)
        if lock is None:
            lock = Lock()
            self._turn_lock = lock
        return lock

    def _stream_pib_cloud_turn(
        self,
        *,
        chat_id: str,
        content: str,
        personality: Any,
        plan: Any,
        generate_code: bool,
        tool_calling: Optional[bool] = None,
        goal_handle=None,
    ):
        """Stream SSE tokens from pib.Cloud (POST /v1/chat/completions, stream=true)."""
        history: list[tuple[str, bool]] = []
        try:
            with self.voice_assistant_client_lock:
                successful, chat_messages = voice_assistant_client.get_chat_history(
                    chat_id, self.history_length
                )
            if successful and chat_messages:
                history = [
                    (message.content, message.is_user) for message in chat_messages
                ]
        except Exception as exc:
            self.get_logger().warning(
                f"pib.Cloud history could not be read for chat {chat_id}: {exc}"
            )

        system_prompt = direct_system_prompt(
            getattr(personality, "description", None),
            images_shown=True,
            robot_context=self._robot_context(),
            code_generation=generate_code,
        )
        system_prompt = plan.for_direct(system_prompt)
        endpoint_base = getattr(
            getattr(personality, "assistant_model", None), "endpoint_base", None
        )
        if tool_calling is None:
            tool_calling = direct_tool_loop.tool_calling_enabled(personality)
        allow_image = direct_tool_loop.image_tool_allowed(personality, tool_calling)
        declarations = direct_tool_loop.mcp_tool_declarations() if tool_calling else []
        tools = direct_tool_loop.tools_for_turn(
            declarations,
            tool_calling=tool_calling,
            allow_image=allow_image,
        )
        return pib_cloud_stream.stream_pib_cloud_chat(
            system_prompt=system_prompt,
            user_text=content,
            history=history,
            chat_id=chat_id,
            endpoint_base=endpoint_base,
            tools=tools,
            allow_image=allow_image,
            on_tool_call=getattr(self, "_record_tool_finished", None),
            on_tool_start=getattr(self, "_record_tool_started", None),
            local_fallback=lambda: self._iter_local_tokens(
                chat_id, content, system_prompt
            ),
            is_cancelled=lambda: bool(
                goal_handle is not None and goal_handle.is_cancel_requested
            ),
        )

    async def chat(self, goal_handle: ServerGoalHandle):
        """
        Action server callback for 'chat':
        - Serializes turns on one pib (OQ12: two turns never run concurrently).
        - Creates an initial user ChatMessage in PIB + publishes it on ROS.
        - Fetches personality + history and streams tokens from pib.Cloud / Hermes / Direct.
        - Splits assistant output into sentences/clauses (with N-token fallback),
          publishing each chunk as feedback and updating the ChatMessage.
        """
        with self._get_turn_lock():
            return self._execute_chat_turn(goal_handle)

    def _execute_chat_turn(self, goal_handle: ServerGoalHandle):
        t0 = time.monotonic()
        self._ensure_turn_trace_anchor(t0)
        self.get_logger().info("start chat request")

        # Unpack request data
        request: Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text
        generate_code: bool = request.generate_code
        turn_id = getattr(request, "turn_id", "") or ""
        self._turn_chat_id = chat_id
        self._turn_trace_turn_id = turn_id
        if turn_id:
            self._emit_turn_state(turn_id, STATE_RUNNING, chat_id)

        self.get_logger().info(
            f"[PERF_TRACE] ROS_SERVICE_RECV chat={chat_id} elapsed_ms=0.00"
        )

        # Create the user message (first chunk) via Action path helper. This
        # write also sets last_pib_message_id, so it has to land before the
        # assistant chunks start creating and updating their own message.
        self.create_chat_message(chat_id, content, True, False, True)

        # Turn toward the detected speaker before any answer, including the
        # degraded-mode sentence. The stored user line stays the words they said.
        plan = self._attention_plan()

        # Get personality (also sets how much history to include)
        with self.voice_assistant_client_lock:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                chat_id
            )
        if not successful or personality is None:
            personality_id = getattr(personality, "personality_id", None)
            if personality_id:
                self.get_logger().error(
                    f"chat could not be resolved chat={chat_id} "
                    f"personality={personality_id}"
                )
            else:
                self.get_logger().error(f"chat could not be resolved chat={chat_id}")
            goal_handle.abort()
            return Chat.Result()
        self.history_length = personality.message_history
        description = personality_system_prompt(
            soul_text=getattr(personality, "description", None),
            robot_context=self._robot_context(),
            code_generation=generate_code,
        )

        # Channel is a personality setting, not the provider's api name.
        channel = turn_channel(personality)
        is_smart = channel == CHANNEL_SMART
        stored_channel = getattr(personality, "channel", None)
        if (
            channel == CHANNEL_DIRECT
            and isinstance(stored_channel, str)
            and stored_channel == CHANNEL_SMART
        ):
            self.get_logger().info(
                f"smart chats disabled; routing chat={chat_id} as direct"
            )
        else:
            self.get_logger().info(f"chat channel={channel} chat={chat_id}")

        api_name = getattr(
            getattr(personality, "assistant_model", None), "api_name", None
        )
        local_model = api_name == LOCAL_MODEL_API_NAME
        pib_cloud_model = api_name == PIB_CLOUD_API_NAME
        model_provider_name = _model_provider_name(api_name)
        tool_calling = direct_tool_loop.tool_calling_enabled(personality)
        # Old Direct text selections stop here, before the key store. A locked
        # or missing key must not hide the retirement reason, and the stored
        # selection is not migrated.
        if legacy_direct_text_selected(
            channel=channel,
            api_name=api_name,
            voice_mode=getattr(personality, "voice_mode", None),
        ):
            return self._finish_retired_direct_text(goal_handle, chat_id, api_name)
        # Only a model whose provider needs a key is gated by the store. A
        # cloud model does; the on-device model does not, and still answers
        # while the store is locked.
        if (
            _model_needs_provider_key(api_name)
            and self._key_store_mode() != MODE_UNLOCKED
        ):
            # Smart and Direct both stop here. The sentence is the personality
            # speaking: the assistant plays it with this personality's gender
            # and language on local Supertone. The goal succeeds.
            if goal_handle.is_cancel_requested:
                self._finish_cancelled_turn(chat_id, goal_handle)
                return Chat.Result()
            sentence = refusal_sentence(channel)
            self.create_chat_message(chat_id, sentence, False, False, True)
            goal_handle.succeed()
            result = Chat.Result()
            result.text = sentence
            result.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
            self.get_logger().info(
                f"chat refused in degraded mode channel={channel} chat={chat_id}"
            )
            return result

        direct_turn_meta: dict[str, Any] | None = None
        try:
            if is_smart:
                turn_path = "Hermes"
                # The channel decides the transport: Smart runs on Hermes, Direct
                # never does. Tool calling says whether a turn offers MCP tools
                # inside whichever runtime serves it - it is not a transport
                # switch, so it must not move a Direct personality onto Hermes.
                # Smart stays on Hermes even for the on-device model: the model
                # decides what Hermes runs, never the transport.
                if goal_handle.is_cancel_requested:
                    self._finish_cancelled_turn(chat_id, goal_handle)
                    return Chat.Result()
                tokens = self._stream_hermes_turn(
                    text=plan.for_hermes(content),
                    chat_id=chat_id,
                    personality_id=getattr(personality, "personality_id", None),
                    description=description,
                    model=api_name,
                    provider=model_provider_name,
                    base_url=_model_base_url(api_name),
                    tool_calling=tool_calling,
                    goal_handle=goal_handle,
                    message_history=self.history_length,
                )
            elif pib_cloud_model:
                turn_path = "pib.Cloud"
                if goal_handle.is_cancel_requested:
                    self._finish_cancelled_turn(chat_id, goal_handle)
                    return Chat.Result()
                self.get_logger().info(
                    f"pib.Cloud streaming chat={chat_id} model={PIB_CLOUD_API_NAME}"
                )
                tokens = self._stream_pib_cloud_turn(
                    chat_id=chat_id,
                    content=content,
                    personality=personality,
                    plan=plan,
                    generate_code=generate_code,
                    tool_calling=tool_calling,
                    goal_handle=goal_handle,
                )
            elif local_model:
                turn_path = "the on-device model"
                system_prompt = direct_system_prompt(
                    personality.description,
                    images_shown=True,
                    robot_context=self._robot_context(),
                    code_generation=generate_code,
                )
                system_prompt = plan.for_direct(system_prompt)
                self.get_logger().info(
                    f"local model chat={chat_id} model={LOCAL_MODEL_API_NAME}"
                )
                tokens = self._iter_local_tokens(chat_id, content, system_prompt)
            else:
                turn_path = "the direct tool loop"
                # Pull recent message history for context
                with self.voice_assistant_client_lock:
                    successful, chat_messages = voice_assistant_client.get_chat_history(
                        chat_id, self.history_length
                    )
                if not successful:
                    self.get_logger().error(
                        f"chat with id'{chat_id}' does not exist..."
                    )
                    goal_handle.abort()
                    return Chat.Result()
                message_history = [
                    PublicApiChatMessage(message.content, message.is_user)
                    for message in chat_messages
                ]

                # Direct: the SOUL text is the system prompt. MEMORY.md is not
                # read; that file belongs to the Hermes profile on Smart turns.
                # A camera frame is never attached here. It arrives only when
                # the model calls capture_image, and that tool is absent while
                # tool calling is off.
                system_prompt = direct_system_prompt(
                    personality.description,
                    images_shown=True,
                    robot_context=self._robot_context(),
                    code_generation=generate_code,
                )
                system_prompt = plan.for_direct(system_prompt)
                allow_image = direct_tool_loop.image_tool_allowed(
                    personality, tool_calling
                )
                api_name = personality.assistant_model.api_name
                if not direct_tool_loop.supports_tool_endpoint(api_name):
                    raise direct_tool_loop.DirectToolLoopError(
                        "Direct tool calling has no non-beta endpoint for "
                        f"model {api_name!r}. The pinned model is "
                        f"{direct_tool_loop.PINNED_MODEL} "
                        f"({direct_tool_loop.PINNED_PROVIDER}), checked "
                        f"{direct_tool_loop.PINNED_CHECKED_ON}."
                    )
                self.get_logger().info(
                    f"direct tool loop model={direct_tool_loop.PINNED_MODEL} "
                    f"provider={direct_tool_loop.PINNED_PROVIDER} chat={chat_id}"
                )
                personality_name = getattr(personality, "name", "") or ""
                direct_turn_meta = {
                    "started": time.monotonic(),
                    "tool_rounds": 0,
                }
                self._turn_direct_tool_rounds = 0
                self.get_logger().info(
                    f"direct turn started personality={personality_name} model={api_name}"
                )

                def count_direct_tool_round() -> None:
                    direct_turn_meta["tool_rounds"] += 1
                    self._turn_direct_tool_rounds = direct_turn_meta["tool_rounds"]

                def report_key_source(source: str) -> None:
                    # source is key-store or environment. The key itself
                    # is not passed here, so it cannot land in the log.
                    self.get_logger().info(
                        f"direct provider key source={source} "
                        f"provider={direct_tool_loop.PINNED_PROVIDER} "
                        f"chat={chat_id}"
                    )

                tokens = direct_tool_loop.run_direct_turn(
                    system_prompt=system_prompt,
                    user_text=content,
                    history=[
                        (message.content, message.is_user)
                        for message in message_history
                    ],
                    tool_calling=tool_calling,
                    allow_image=allow_image,
                    report_key_source=report_key_source,
                    on_tool_call=self._record_tool_finished,
                    on_tool_start=self._record_tool_started,
                    on_tool_image=self._record_tool_image,
                    on_tool_round=count_direct_tool_round,
                    session=self._direct_session(personality),
                    # Identity edits (description/SOUL) must replace the frozen prompt.
                    session_identity=self._direct_identity_fingerprint(personality),
                )

            # Which runtime serves this turn, before the first token is asked
            # for: a UI needs it even when the transport fails afterwards.
            self._emit_turn_path(chat_id=chat_id, path=turn_path, model=api_name)
            prev_text, prev_text_type, curr_text = self._stream_chunks_to_goal(
                goal_handle, chat_id, tokens, t0=t0
            )
            if direct_turn_meta is not None:
                try:
                    elapsed = time.monotonic() - direct_turn_meta["started"]
                    self.get_logger().info(
                        f"direct turn finished in {elapsed:.1f} s, "
                        f"{direct_turn_meta['tool_rounds']} tool rounds"
                    )
                except Exception:
                    pass
            if goal_handle.is_cancel_requested:
                self._finish_cancelled_turn(chat_id, goal_handle)
                return Chat.Result()
            self._finalize_message_meta(chat_id)
            if turn_id:
                self._emit_turn_state(turn_id, STATE_COMPLETED, chat_id)

        except Exception as e:
            # Name the path the turn actually took: a pib.Cloud turn with tools
            # enabled runs on Hermes, and a log that still says "direct" sends
            # the next diagnosis to the wrong component.
            self.get_logger().error(f"failed to send request to {turn_path}: {e}")
            goal_handle.abort()
            return Chat.Result()

        # Finish Action: return the last pending chunk (if any)
        goal_handle.succeed()
        result = Chat.Result()
        if prev_text is None:
            result.text = curr_text
            result.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
        else:
            result.text = prev_text
            result.text_type = prev_text_type
        self.get_logger().info(
            f"[PERF_TRACE] ROS_SERVICE_DONE chat={chat_id} "
            f"elapsed_ms={(time.monotonic() - t0) * 1000.0:.2f}"
        )
        return result


def main(args=None):
    """
    Standard ROS 2 entrypoint:
    - Starts ChatNode with a MultiThreadedExecutor (8 threads).
    - Spins forever until shutdown.
    """
    rclpy.init()
    node = ChatNode()
    executor = MultiThreadedExecutor(8)  # chosen arbitrarily, allows concurrent goals
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
