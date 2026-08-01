import os
import re
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
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

from public_api_client import hermes_agent_client, public_voice_client

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
    - Exposes a ROS 2 Action "chat" for request/response (token streaming) via public_api.
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
            self.get_logger().error("VisionPrompt failed: public_api_token is not available.")
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
                    self.get_logger().error("VisionPrompt: get_camera_image service unavailable.")
                    resp.response = "0"
                    return resp

                camera_future = tmp_client.call_async(camera_request)
                rclpy.spin_until_future_complete(
                    tmp_node,
                    camera_future,
                    timeout_sec=5.0,
                )

                if not camera_future.done():
                    self.get_logger().error("VisionPrompt: get_camera_image request timed out.")
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
        self, goal_handle, chat_id: str, tokens
    ) -> tuple[Optional[str], Optional[int], str]:
        """Consume a token iterable, publishing sentence/code chunks as feedback.

        Returns (prev_text, prev_text_type, curr_text) so the caller can build Chat.Result.
        Behavior is identical to the previous inline implementation.
        """
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

        for token in tokens:
            # Publish previous chunk as feedback (Action protocol)
            if prev_text is not None:
                feedback = Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

            # Accumulate token (strip leading spaces if first)
            curr_text = curr_text + (
                token if len(curr_text) > 0 else token.lstrip()
            )

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
            if personality_id:
                hermes_agent_client.ensure_profile(
                    personality_id, soul_text=description
                )
            return hermes_agent_client.run_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
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
                # The worker is left to finish on its own; it only touches the
                # hermes CLI, and the pool outlives this goal, so nothing here
                # breaks when the subprocess returns later.
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

    async def chat(self, goal_handle: ServerGoalHandle):
        """
        Action server callback for 'chat':
        - Creates an initial user ChatMessage in PIB + publishes it on ROS.
        - Fetches personality + history and streams tokens from public_api.
        - Splits assistant output into sentences (and <pib-program> blocks),
          publishing each chunk as an updated ChatMessage via create_chat_message().
        """
        self.get_logger().info("start chat request")

        # Unpack request data
        request: Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text
        generate_code: bool = request.generate_code

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

        is_hermes = hermes_agent_client.uses_hermes_backend(
            personality.assistant_model.api_name
        )

        try:
            if is_hermes:
                reply_text = self._run_hermes_turn(
                    text=content,
                    chat_id=chat_id,
                    personality_id=getattr(personality, "personality_id", None),
                    description=description,
                    goal_handle=goal_handle,
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Chat.Result()
                tokens = [reply_text]
            else:
                # Pull recent message history for context
                with self.voice_assistant_client_lock:
                    successful, chat_messages = voice_assistant_client.get_chat_history(
                        chat_id, self.history_length
                    )
                if not successful:
                    self.get_logger().error(f"chat with id'{chat_id}' does not exist...")
                    goal_handle.abort()
                    return Chat.Result()
                message_history = [
                    PublicApiChatMessage(message.content, message.is_user)
                    for message in chat_messages
                ]

                # get the current image from the camera if available
                image_base64 = None
                if personality.assistant_model.has_image_support:
                    if not self.get_camera_image_client.service_is_ready():
                        self.get_logger().warn(
                            "get_camera_image service is not ready, proceeding without image."
                        )
                        image_base64 = None
                    else:
                        request = GetCameraImage.Request()
                        try:
                            future = self.get_camera_image_client.call_async(request)
                            response = await future
                            image_base64 = response.image_base64
                        except Exception as e:
                            self.get_logger().error(f"Camera service call failed: {e}")
                            image_base64 = None

                # Stream tokens from public API (yields text tokens)
                with self.public_voice_client_lock:
                    tokens = public_voice_client.chat_completion(
                        text=content,
                        description=description,
                        message_history=message_history,
                        image_base64=image_base64,
                        model=personality.assistant_model.api_name,
                        public_api_token=self.token,
                    )

            prev_text, prev_text_type, curr_text = self._stream_chunks_to_goal(
                goal_handle, chat_id, tokens
            )
            if goal_handle.is_cancel_requested:
                return Chat.Result()

        except Exception as e:
            backend = "hermes-agent" if is_hermes else "public-api"
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
