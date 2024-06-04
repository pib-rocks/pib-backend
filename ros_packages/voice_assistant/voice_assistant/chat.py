import re
from threading import Lock

import rclpy
from datatypes.action import Chat
from datatypes.msg import ChatMessage
from datatypes.srv import GetCameraImage
from pib_api_client import voice_assistant_client
from public_api_client.public_voice_client import PublicApiChatMessage
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from public_api_client import public_voice_client


class ChatNode(Node):

    def __init__(self):

        super().__init__("chat")

        # server for communicating with an llm via tryb's public-api
        # In the goal, a client specifies some text that will be sent as input to the llm, as well as the
        # description of the personality. The server then forwards the llm output to the client at the
        # granularity of sentences. Intermediate sentences, are forwared in form of feedback. The final
        # sentence is forwarded as the result of the goal
        self.chat_server = ActionServer(
            self,
            Chat,
            "chat",
            execute_callback=self.chat,
            cancel_callback=(lambda _: CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup(),
        )

        # Publisher for ChatMessages
        self.chat_message_publisher: Publisher = self.create_publisher(
            ChatMessage, "chat_messages", 10
        )
        self.camera_client = self.create_client(GetCameraImage, "get_camera_image")

        # lock that should be aquired, whenever accessing 'public_voice_client'
        self.public_voice_client_lock = Lock()
        # lock that should be aquired, whenever accessing 'voice_assistant_client'
        self.voice_assistant_client_lock = Lock()

        self.get_logger().info("Now running CHAT")

    def create_chat_message(self, chat_id: str, text: str, is_user: bool) -> None:
        """writes a new chat-message to the db, and publishes it to the 'chat_messages'-topic"""

        if text == "":
            return

        with self.voice_assistant_client_lock:
            successful, chat_message = voice_assistant_client.create_chat_message(
                chat_id, text, is_user
            )
        if not successful:
            self.get_logger().error(
                f"unable to create chat message: {(chat_id, text, is_user)}"
            )
            return

        chat_message_ros = ChatMessage()
        chat_message_ros.chat_id = chat_id
        chat_message_ros.content = chat_message.content
        chat_message_ros.is_user = chat_message.is_user
        chat_message_ros.message_id = chat_message.message_id
        chat_message_ros.timestamp = chat_message.timestamp

        self.chat_message_publisher.publish(chat_message_ros)

    async def chat(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("start chat request")
        # unpack request data
        request: Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text

        # get the personality that is associated with the request chat_id from the pib-api
        with self.voice_assistant_client_lock:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                chat_id
            )
        if not successful:
            self.get_logger().error(f"no personality found for id {chat_id}")
            goal_handle.abort()
            return Chat.Result()

        # create the user message
        self.executor.create_task(self.create_chat_message, chat_id, content, True)

        # receive an iterable of tokens from the public-api
        description = (
            personality.description
            if personality.description is not None
            else "Du bist pib, ein humanoider Roboter."
        )
        camera_response = None
        if personality.assistant_model.has_image_support:
            camera_response_future = await self.camera_client.call_async(
                GetCameraImage.Request()
            )
            camera_response = camera_response_future.image_base64

        with self.voice_assistant_client_lock:
            successful, chat_messages = voice_assistant_client.get_all_chat_messages(
                chat_id
            )
        if not successful:
            self.get_logger().error(f"chat with id'{chat_id}' does not exist...")
            goal_handle.abort()
            return Chat.Result()

        message_history = [
            PublicApiChatMessage(message.content, message.is_user)
            for message in chat_messages
        ]

        with self.public_voice_client_lock:
            try:
                tokens = public_voice_client.chat_completion(
                    text=content,
                    description=description,
                    message_history=message_history,
                    image_base64=camera_response,
                    model=personality.assistant_model.api_name,
                )

                curr_sentence: str = ""
                prev_sentence: str | None = None
                sentence_boundary = re.compile(r"[^\d | ^A-Z][\.|!|\?|:]")

                for token in tokens:
                    # if the goal was cancelled, return immediately
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return Chat.Result(rest=curr_sentence)
                    # if a sentence was already found and another token was received, forward the sentence as feedback
                    if prev_sentence is not None:
                        self.executor.create_task(
                            self.create_chat_message, chat_id, prev_sentence, False
                        )
                        feedback = Chat.Feedback()
                        feedback.sentence = prev_sentence
                        goal_handle.publish_feedback(feedback)
                        prev_sentence = None
                    # check if the current token marks the end of a sentence
                    if sentence_boundary.search(curr_sentence):
                        prev_sentence = curr_sentence.strip()
                        curr_sentence = ""
                    curr_sentence += token
            except Exception as e:
                self.get_logger().error(f"chat completion failed: {e}")
                goal_handle.abort()
                return Chat.Result()

        # create chat-message for remaining input
        if len(curr_sentence) > 0:
            self.executor.create_task(
                self.create_chat_message, chat_id, curr_sentence, False
            )

        # return the restult
        result = Chat.Result()
        result.rest = curr_sentence if prev_sentence is None else prev_sentence
        goal_handle.succeed()
        return result


def main(args=None):

    rclpy.init()
    node = ChatNode()
    # the number of threads is chosen arbitrarily to be '8' because ros requires a
    # fixed number of threads to be specified. Generally, multiple goals should
    # be handled simultaneously, so the number should be sufficiently large.
    executor = MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
