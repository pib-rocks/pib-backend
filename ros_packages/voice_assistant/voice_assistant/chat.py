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

from voice_assistant.chat_turn_trace_mixin import ChatTurnTraceMixin
from voice_assistant.chat_hermes_mixin import ChatHermesMixin

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


class ChatNode(ChatTurnTraceMixin, ChatHermesMixin, Node):
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
        *,
        accumulated: bool = False,
    ) -> None:
        """
        Writes a new chat-message (or updates the last one) to PIB DB,
        and publishes it on the 'chat_messages' topic.

        - When update_message=False → CREATE new message in PIB (records message_id)
        - When update_message=True  → UPDATE existing PIB message_id with concatenated content
        - update_database controls whether we hit PIB on updates or only update local content
        - accumulated=True → ``text`` is the full assistant answer so far (streaming turns)
        """
        if text == "":
            return

        with self.voice_assistant_client_lock:
            if update_message:
                # UPDATE path
                if update_database:
                    # concatenate locally AND persist to PIB
                    if accumulated:
                        self.message_content = text
                    else:
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
                    if accumulated:
                        self.message_content = text
                    else:
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

    def _get_turn_lock(self) -> Lock:
        """Return the per-node turn serialization lock (created lazily for test doubles)."""
        lock = getattr(self, "_turn_lock", None)
        if lock is None:
            lock = Lock()
            self._turn_lock = lock
        return lock

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
