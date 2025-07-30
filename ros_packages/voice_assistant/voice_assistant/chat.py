import re
import asyncio
from threading import Lock
from typing import Optional, Iterable

import rclpy
import datetime
from datatypes.action import Chat
from datatypes.msg import ChatMessage
from datatypes.srv import GetCameraImage
from public_api_client.public_voice_client import PublicApiChatMessage
from pib_api_client import voice_assistant_client
from .chat_factory import chat_completion as factory_chat_completion
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, AsyncIOExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

# In future, this prefix can contain instructions for code generation.
CODE_DESCRIPTION_PREFIX = ""


class ChatNode(Node):
    def __init__(self):
        super().__init__("chat")

        # token for public‐API / Gemini Live API (via TokenServiceNode)
        self.token: Optional[str] = None

        # history settings
        self.last_pib_message_id: Optional[str] = None
        self.message_content: Optional[str] = None
        self.history_length: int = 10

        # ActionServer for Chat requests
        self.chat_server = ActionServer(
            self,
            Chat,
            "chat",
            execute_callback=self.chat,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

        # Publisher for incremental ChatMessages
        self.chat_message_publisher: Publisher = self.create_publisher(
            ChatMessage, "chat_messages", 10
        )

        # Service client for optional camera images
        self.get_camera_image_client = self.create_client(
            GetCameraImage, "get_camera_image"
        )

        # Subscribe to incoming public-API token
        self.get_token_subscription = self.create_subscription(
            String, "public_api_token", self.get_public_api_token_listener, 10
        )

        # Locks to protect shared clients
        self.public_voice_client_lock = Lock()
        self.voice_assistant_client_lock = Lock()

        self.get_logger().info("Now running CHAT")

    def get_public_api_token_listener(self, msg: String):
        self.token = msg.data

    def create_chat_message(
        self,
        chat_id: str,
        text: str,
        is_user: bool,
        update_message: bool,
        update_database: bool,
    ) -> None:
        """Persist & publish a ChatMessage (user or assistant)."""
        if not text:
            return

        # Update or create via PIB API
        with self.voice_assistant_client_lock:
            if update_message:
                if update_database:
                    self.message_content += " " + text
                    successful, chat_msg = voice_assistant_client.update_chat_message(
                        chat_id,
                        self.message_content,
                        is_user,
                        self.last_pib_message_id,
                    )
                else:
                    self.message_content += " " + text
                    successful, chat_msg = True, None
            else:
                successful, chat_msg = voice_assistant_client.create_chat_message(
                    chat_id, text, is_user
                )
                self.last_pib_message_id = chat_msg.message_id
                self.message_content = text

            if not successful:
                self.get_logger().error(
                    f"ChatMessage persist failed: {(chat_id, text, is_user)}"
                )
                return

        # Publish over ROS
        ros_msg = ChatMessage()
        ros_msg.chat_id = chat_id
        ros_msg.content = self.message_content
        ros_msg.is_user = is_user
        ros_msg.message_id = self.last_pib_message_id or ""
        ros_msg.timestamp = datetime.datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        self.chat_message_publisher.publish(ros_msg)

    async def chat(self, goal_handle):
        """Handle incoming Chat action goals."""
        self.get_logger().info("Start chat request")
        req: Chat.Goal = goal_handle.request
        chat_id = req.chat_id
        content = req.text
        generate_code = req.generate_code

        # 1) Publish user message immediately
        self.create_chat_message(chat_id, content, True, False, True)

        # 2) Fetch personality & history from PIB API
        with self.voice_assistant_client_lock:
            ok, personality = voice_assistant_client.get_personality_from_chat(chat_id)
        if not ok:
            self.get_logger().error(f"No personality for chat_id={chat_id}")
            goal_handle.abort()
            return Chat.Result()

        description = personality.description or "Du bist pib, ein humanoider Roboter."
        if generate_code:
            description = CODE_DESCRIPTION_PREFIX + description
        self.history_length = personality.message_history

        with self.voice_assistant_client_lock:
            ok, history = voice_assistant_client.get_chat_history(
                chat_id, self.history_length
            )
        if not ok:
            self.get_logger().error(f"Chat history missing for {chat_id}")
            goal_handle.abort()
            return Chat.Result()

        message_history = [
            PublicApiChatMessage(m.content, m.is_user) for m in history
        ]

        # 3) Optionally grab a camera image
        image_base64 = None
        if personality.assistant_model.has_image_support:
            resp = await self.get_camera_image_client.call_async(
                GetCameraImage.Request()
            )
            image_base64 = resp.image_base64

        # regex to split sentences vs code blocks
        SENTENCE_RE = re.compile(
            r"^(?!<pib-program>)(.*?)(([^\d | ^A-Z][\.!?])|<pib-program>)", re.DOTALL
        )
        CODE_RE = re.compile(r"^<pib-program>(.*?)</pib-program>", re.DOTALL)

        curr_text = ""
        prev_text = None
        prev_type = None
        update_db = False

        try:
            # stream text tokens via the public API
            async with self.public_voice_client_lock:
                token_iter = factory_chat_completion(
                    text=content,
                    description=description,
                    message_history=message_history,
                    image_base64=image_base64,
                    model=personality.assistant_model.api_name,
                    public_api_token=self.token,
                )

            # stream tokens
            async for token in token_iter:
                # handle cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Chat.Result()

                # accumulate & split into sentences or code
                curr_text += token if curr_text else token.lstrip()

                while True:
                    code_match = CODE_RE.search(curr_text)
                    if code_match:
                        prev_text = code_match.group(1)
                        prev_type = Chat.Goal.TEXT_TYPE_CODE_VISUAL
                        self.create_chat_message(
                            chat_id,
                            f"<pib-program>{prev_text}</pib-program>",
                            False,
                            update_db,
                            True,
                        )
                        curr_text = curr_text[code_match.end():].lstrip()
                        update_db = True
                        continue

                    sent_match = SENTENCE_RE.search(curr_text)
                    if sent_match:
                        prev_text = sent_match.group(1) + (sent_match.group(3) or "")
                        prev_type = Chat.Goal.TEXT_TYPE_SENTENCE
                        self.create_chat_message(
                            chat_id, prev_text, False, update_db, True
                        )
                        curr_text = curr_text[sent_match.end():].lstrip()
                        update_db = True
                        continue

                    break

            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f"Chat streaming failed: {e}")
            goal_handle.abort()
            return Chat.Result()

        # final leftover
        result = Chat.Result()
        if prev_text is None:
            result.text = curr_text
            result.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
        else:
            result.text = prev_text
            result.text_type = prev_type
        return result


def main(args=None):
    rclpy.init()
    node = ChatNode()
    executor = AsyncIOExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
