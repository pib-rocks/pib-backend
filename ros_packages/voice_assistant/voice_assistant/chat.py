import re
from threading import Lock
from typing import Optional

import rclpy
import datetime
from datatypes.action import Chat
from datatypes.msg import ChatMessage
from datatypes.srv import GetCameraImage

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

from public_api_client import public_voice_client

# In future, this code will be prepended to the description in a chat-request
# if it is specified that code should be generated. The text will contain
# instruction for the LLM on how to generate the code. For now, it is left blank.
CODE_DESCRIPTION_PREFIX = ""


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

        self.get_logger().info("Now running CHAT")

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

    # ---------- Action server (unchanged) ----------

    def get_public_api_token_listener(self, msg):
        """Receives the token for public_api via ROS topic 'public_api_token'."""
        token = msg.data
        self.token = token

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

        # Create the user message (first chunk) via Action path helper
        self.executor.create_task(
            self.create_chat_message, chat_id, content, True, False, True
        )

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

        # Optional camera image context
        image_base64 = None
        if personality.assistant_model.has_image_support:
            response: GetCameraImage.Response = (
                await self.get_camera_image_client.call_async(GetCameraImage.Request())
            )
            image_base64 = response.image_base64

        try:
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
                        return Chat.Result()

                    # Visual code block
                    code_visual_match = code_visual_pattern.search(curr_text)
                    if code_visual_match is not None:
                        code_visual = code_visual_match.group(1)
                        prev_text = code_visual
                        prev_text_type = Chat.Goal.TEXT_TYPE_CODE_VISUAL
                        chat_message_text = code_visual_match.group(0)
                        self.executor.create_task(
                            self.create_chat_message,
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
                        self.executor.create_task(
                            self.create_chat_message,
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

        except Exception as e:
            self.get_logger().error(f"failed to send request to public-api: {e}")
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
