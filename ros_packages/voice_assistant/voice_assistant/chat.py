import os
import re
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from queue import Empty, Queue
from threading import Lock
from typing import Optional

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
from std_msgs.msg import String

from pib_hermes_config.turn_taking import unpublished_clauses
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    direct_system_prompt,
    turn_channel,
)
from public_api_client import hermes_agent_client, public_voice_client
from voice_assistant import direct_tool_loop
from voice_assistant.degraded_chat import (
    MODE_UNLOCKED,
    fetch_operating_mode,
    refusal_sentence,
)

# In future, this code will be prepended to the description in a chat-request
# if it is specified that code should be generated. The text will contain
# instruction for the LLM on how to generate the code. For now, it is left blank.
CODE_DESCRIPTION_PREFIX = ""

# How often the hermes turn is interrupted to notice a cancel request.
HERMES_CANCEL_POLL_SECONDS = 0.2

# Slack on top of the hermes turn's own subprocess timeout, so a wedged worker
# can never hold a goal open forever.
HERMES_WAIT_GRACE_SECONDS = 15

# Upper bound for the startup liveness probe. Node construction blocks on it, so
# it stays short; a failed probe only downgrades hermes personalities to the
# fallback reply and must never keep the node from starting.
HERMES_PROBE_TIMEOUT_SECONDS = 5


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

        self.get_logger().info("Now running CHAT")

    def _key_store_mode(self) -> str:
        """Operating mode of the key store. Unreadable means degraded."""
        return fetch_operating_mode()

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
        healthy while every hermes-agent personality quietly answers with the
        fallback sentence. Legacy personalities are unaffected, so this only logs.

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
            "Personalities using the 'hermes-agent' model will fall back to a "
            "canned reply. Check that the hermes CLI is installed for the pib "
            "user, that PIB_HERMES_BIN points at it, and that ~/.hermes, the "
            "wrapper and the uv-managed Python directory are all bind-mounted "
            "into the ros-voice-assistant service."
        )
        return False

    # ---------- common helpers used by Action and Service ----------

    def _publish_chat_message(
        self, chat_id: str, content: str, is_user: bool, message_id: str
    ):
        """
        Build and publish a datatypes/ChatMessage with current timestamp.
        Used by both the action path (create_chat_message) and the service path.
        """
        msg = ChatMessage()
        msg.chat_id = chat_id
        msg.content = content
        msg.is_user = is_user
        msg.message_id = message_id
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
                    successful, _ = voice_assistant_client.update_chat_message(
                        chat_id,
                        self.message_content,
                        is_user,
                        self.last_pib_message_id,
                    )
                    if not successful:
                        self.get_logger().error(
                            f"unable to create chat message: {(chat_id, text, is_user, update_message, update_database)}"
                        )
                        return
                else:
                    # concatenate locally ONLY
                    self.message_content = f"{self.message_content} {text}"
            else:
                # CREATE path
                successful, chat_message = voice_assistant_client.create_chat_message(
                    chat_id, text, is_user
                )
                if not successful or chat_message is None:
                    self.get_logger().error(
                        f"unable to create chat message: {(chat_id, text, is_user, update_message, update_database)}"
                    )
                    return
                self.last_pib_message_id = chat_message.message_id
                self.message_content = text

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

        if self.token is None:
            self.get_logger().error(
                "VisionPrompt failed: public_api_token is not available."
            )
            resp.response = "0"
            return resp

        image_base64 = None

        if not self.get_camera_image_client.service_is_ready():
            self.get_logger().warn(
                "VisionPrompt: get_camera_image service is not ready."
            )
            resp.response = "0"
            return resp

        try:
            camera_request = GetCameraImage.Request()

            tmp_node = rclpy.create_node("vision_prompt_camera_client")
            try:
                tmp_client = tmp_node.create_client(GetCameraImage, "get_camera_image")

                if not tmp_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error(
                        "VisionPrompt: get_camera_image service unavailable."
                    )
                    resp.response = "0"
                    return resp

                camera_future = tmp_client.call_async(camera_request)
                rclpy.spin_until_future_complete(
                    tmp_node,
                    camera_future,
                    timeout_sec=5.0,
                )

                if not camera_future.done():
                    self.get_logger().error(
                        "VisionPrompt: get_camera_image request timed out."
                    )
                    resp.response = "0"
                    return resp

                camera_response = camera_future.result()
            finally:
                tmp_node.destroy_node()

            if camera_response is None or not camera_response.image_base64:
                self.get_logger().warn("VisionPrompt: camera returned no image.")
                resp.response = "0"
                return resp

            image_base64 = camera_response.image_base64

        except Exception as exc:
            self.get_logger().error(f"VisionPrompt camera request failed: {exc}")
            resp.response = "0"
            return resp

        try:
            with self.public_voice_client_lock:
                tokens = public_voice_client.chat_completion(
                    text=prompt,
                    description=(
                        "Du bist ein Vision-Erkennungsmodul fuer Blockly. "
                        "Befolge das verlangte Antwortformat exakt."
                    ),
                    message_history=[],
                    image_base64=image_base64,
                    model="gpt-4o",
                    public_api_token=self.token,
                )

                response_text = "".join(tokens).strip()

            resp.response = response_text
            return resp

        except Exception as exc:
            self.get_logger().error(f"VisionPrompt public API request failed: {exc}")
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

        For TTFT, the first non-empty token is published as Action feedback
        immediately (before sentence buffering completes) so downstream TTS can
        start without waiting for a terminator.
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
                    self.get_logger().info(
                        f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                        f"elapsed_ms={elapsed_ms:.2f}"
                    )

            # Publish previous completed chunk as feedback (Action protocol)
            if prev_text is not None:
                feedback = Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                if prev_text_type == Chat.Goal.TEXT_TYPE_SENTENCE:
                    published_speech.append(prev_text)
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

            self._publish_ready_clauses(goal_handle, raw_answer, published_speech)

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
                self.get_logger().info(
                    f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                    f"elapsed_ms={elapsed_ms:.2f}"
                )
            self._publish_ready_clauses(goal_handle, raw_answer, published_speech)
            if prev_text is not None:
                # Hand the completed chunk over as feedback the way the next
                # token would have, so the tail can travel in Chat.Result.
                feedback = Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

        return prev_text, prev_text_type, curr_text

    def _remember_first_token_latency(self, chat_id: str, elapsed_ms: float) -> None:
        """Persist the measurement. A down API must not fail the turn."""
        try:
            voice_assistant_client.record_first_token_latency(chat_id, elapsed_ms)
        except Exception as exc:
            self.get_logger().warning(
                "first-token latency was not stored for chat %s: %s",
                chat_id,
                exc,
            )

    def _publish_ready_clauses(
        self, goal_handle, raw_answer: str, published_speech: list[str]
    ) -> None:
        """Publish each newly completed clause so speech can start on it."""
        for clause in unpublished_clauses(raw_answer, published_speech):
            feedback = Chat.Feedback()
            feedback.text = clause
            feedback.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
            goal_handle.publish_feedback(feedback)
            published_speech.append(clause)

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
            # Hermes keeps its own durable memory per chat, so no history is
            # replayed: persona comes from the profile (-p), memory from the
            # named session (-c).
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
                        personality_id, soul_text=description
                    )
            return hermes_agent_client.run_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                toolsets=hermes_agent_client.DEFAULT_DISABLED_TOOLSETS,
                enabled_toolsets=hermes_agent_client.DEFAULT_ENABLED_TOOLSETS,
                timeout=timeout,
            )

        future = self._hermes_executor.submit(_turn)
        deadline = time.monotonic() + timeout + HERMES_WAIT_GRACE_SECONDS

        while True:
            try:
                return future.result(timeout=HERMES_CANCEL_POLL_SECONDS)
            except FutureTimeoutError:
                pass
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
                            personality_id, soul_text=description
                        )
                for delta in hermes_agent_client.stream_turn(
                    text=text,
                    chat_id=chat_id,
                    personality_id=personality_id,
                    toolsets=hermes_agent_client.DEFAULT_DISABLED_TOOLSETS,
                    enabled_toolsets=hermes_agent_client.DEFAULT_ENABLED_TOOLSETS,
                    timeout=timeout,
                ):
                    events.put(("delta", delta))
                events.put(("done", None))
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
            )
            if emitted and reply.startswith(emitted):
                reply = reply[len(emitted) :]
            if reply:
                yield reply
            return

    async def chat(self, goal_handle: ServerGoalHandle):
        """
        Action server callback for 'chat':
        - Creates an initial user ChatMessage in PIB + publishes it on ROS.
        - Fetches personality + history and streams tokens from public_api.
        - Splits assistant output into sentences (and <pib-program> blocks),
          publishing each chunk as an updated ChatMessage via create_chat_message().
        """
        t0 = time.monotonic()
        self.get_logger().info("start chat request")

        # Unpack request data
        request: Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text
        generate_code: bool = request.generate_code

        self.get_logger().info(
            f"[PERF_TRACE] ROS_SERVICE_RECV chat={chat_id} elapsed_ms=0.00"
        )

        # Create the user message (first chunk) via Action path helper. This
        # write also sets last_pib_message_id, so it has to land before the
        # assistant chunks start creating and updating their own message.
        self.create_chat_message(chat_id, content, True, False, True)

        # Get personality (also sets how much history to include)
        with self.voice_assistant_client_lock:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                chat_id
            )
            self.history_length = personality.message_history
        if not successful:
            self.get_logger().error(f"no personality found for id {chat_id}")
            goal_handle.abort()
            return Chat.Result()
        description = (
            personality.description
            if personality.description is not None
            else "Du bist pib, ein humanoider Roboter."
        )
        if generate_code:
            description = CODE_DESCRIPTION_PREFIX + description

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

        if self._key_store_mode() != MODE_UNLOCKED:
            # Smart and Direct both stop here. The sentence is the personality
            # speaking: the assistant plays it with this personality's gender
            # and language on local Supertone. The goal succeeds.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
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

        try:
            if is_smart:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Chat.Result()
                tokens = self._stream_hermes_turn(
                    text=content,
                    chat_id=chat_id,
                    personality_id=getattr(personality, "personality_id", None),
                    description=description,
                    goal_handle=goal_handle,
                )
            else:
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
                system_prompt = direct_system_prompt(personality.description)
                if generate_code:
                    system_prompt = CODE_DESCRIPTION_PREFIX + system_prompt
                tool_calling = direct_tool_loop.tool_calling_enabled(personality)
                allow_image = direct_tool_loop.image_tool_allowed(
                    personality, tool_calling
                )
                api_name = personality.assistant_model.api_name
                if tool_calling:
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
                    tokens = direct_tool_loop.run_direct_turn(
                        system_prompt=system_prompt,
                        user_text=content,
                        history=[
                            (message.content, message.is_user)
                            for message in message_history
                        ],
                        tool_calling=True,
                        allow_image=allow_image,
                    )
                else:
                    with self.public_voice_client_lock:
                        tokens = public_voice_client.chat_completion(
                            text=content,
                            description=system_prompt,
                            message_history=message_history,
                            image_base64=None,
                            model=api_name,
                            public_api_token=self.token,
                        )

            prev_text, prev_text_type, curr_text = self._stream_chunks_to_goal(
                goal_handle, chat_id, tokens, t0=t0
            )
            if goal_handle.is_cancel_requested:
                return Chat.Result()

        except Exception as e:
            backend = "hermes-agent" if is_smart else "direct"
            self.get_logger().error(f"failed to send request to {backend}: {e}")
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
