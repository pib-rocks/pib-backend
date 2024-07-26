import re
from threading import Lock
from typing import Optional

import rclpy
import datetime
from datatypes.action import Chat
from datatypes.msg import ChatMessage as RosChatMessage
from datatypes.srv import GetCameraImage
from pib_api_client import voice_assistant_client
from pib_api_client.voice_assistant_client import ChatMessage as PibApiChatMessage
from public_api_client.public_voice_client import PublicApiChatMessage
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from public_api_client import public_voice_client

# in future, this code will be prepended to the description in a chat-request
# if it is specified that code should be generated. The text will contain
# instruction for the llm on how to generate the code. For now, it is left blank
CODE_DESCRIPTION_PREFIX = ""


class ChatNode(Node):

    def __init__(self):

        super().__init__("chat")
        self.token: Optional[str] = None
        self.last_pib_message_id: Optional[str] = None
        self.message_content: Optional[str] = None
        self.history_length: int = 20

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
            RosChatMessage, "chat_messages", 10
        )
        self.get_camera_image_client = self.create_client(
            GetCameraImage, "get_camera_image"
        )
        self.get_token_subscription = self.create_subscription(
            String, "public_api_token", self.get_public_api_token_listener, 10
        )

        # lock that should be aquired, whenever accessing 'public_voice_client'
        self.public_voice_client_lock = Lock()
        # lock that should be aquired, whenever accessing 'voice_assistant_client'
        self.voice_assistant_client_lock = Lock()

        self.get_logger().info("Now running CHAT")

    def get_public_api_token_listener(self, msg: String) -> None:
        token = msg.data
        self.token = token

    def publish_chat_message(self, pib_api_chat_message: PibApiChatMessage, chat_id: str) -> None:
        chat_message_ros = RosChatMessage()
        chat_message_ros.chat_id = chat_id
        chat_message_ros.content = pib_api_chat_message.content
        chat_message_ros.is_user = pib_api_chat_message.is_user
        chat_message_ros.message_id = pib_api_chat_message.message_id
        chat_message_ros.timestamp = pib_api_chat_message.timestamp
        self.chat_message_publisher.publish(chat_message_ros)

    def update_chat_message(self, chat_id: str, message_id: str, delta: str):
        """updates a chat-message in the db and publishes the updated method to the 'chat_messages'-topic"""
        with self.voice_assistant_client_lock:
            successful, chat_message = (
                voice_assistant_client.extend_chat_message(
                    chat_id,
                    message_id,
                    delta,
                )
            )
            if not successful:
                self.get_logger().error(
                    f"unable to update chat message: {(chat_id, message_id, delta)}"
                )
                return
            self.publish_chat_message(chat_message)

    def create_chat_message(self, chat_id: str, text: str, is_user: bool) -> str:
        """writes a new chat-message to the db, and publishes it to the 'chat_messages'-topic"""
        if text == "": # TODO
            return
        with self.voice_assistant_client_lock:
            successful, chat_message = voice_assistant_client.create_chat_message(
                chat_id, 
                text, 
                is_user
            )
            if not successful:
                self.get_logger().error(
                    f"unable to create chat message: {(chat_id, text, is_user)}"
                )
                return
        self.publish_chat_message(chat_message)
        return chat_message.message_id
    
    async def chat(self, goal_handle: ServerGoalHandle):
        """callback function for 'chat'-action"""

        self.get_logger().info("start chat request")

        # unpack request data
        request: Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text
        generate_code: bool = request.generate_code

        # create the user message
        self.executor.create_task( #######################
            self.create_chat_message, chat_id, content, True
        )

        # get the personality that is associated with the request chat-id from the pib-api
        with self.voice_assistant_client_lock:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                chat_id
            )
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

        # get the message-history from the pib-api
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

        # get the current image from the camera
        image_base64 = None
        if personality.assistant_model.has_image_support:
            response: GetCameraImage.Response = (
                await self.get_camera_image_client.call_async(GetCameraImage.Request())
            )
            image_base64 = response.image_base64

        try:
            # receive assistant-response in form of an iterable of tokens from the public-api
            with self.public_voice_client_lock:
                tokens = public_voice_client.chat_completion(
                    text=content,
                    description=description,
                    message_history=message_history,
                    image_base64=image_base64,
                    model=personality.assistant_model.api_name,
                    public_api_token=self.token,
                )

            # regex for indentifying sentences
            sentence_pattern = re.compile(
                r"^(?!<pib-program>)(.*?)(([^\d | ^A-Z][\.|!|\?|:])|<pib-program>)",
                re.DOTALL,
            )
            # regex-pattern for indentifying visual-code blocks
            code_visual_pattern = re.compile(
                r"^<pib-program>(.*?)</pib-program>", re.DOTALL
            )

            # the text that was currently collected by chaining together tokens
            # at any given point in time, this string must not contain any leading whitespaces!
            curr_text: str = ""
            # previously collected text, that is waiting to be published as feedback
            prev_text: Optional[str] = None
            # for tracking if a message is an update or a new message
            message_id: Optional[str] = None

            for token in tokens:

                if prev_text is not None:
                    # publish the previously collected text in form of feedback
                    feedback = Chat.Feedback()
                    feedback.text = prev_text
                    feedback.text_type = prev_text_type
                    goal_handle.publish_feedback(feedback)
                    prev_text = None
                    prev_text_type = None

                # add token to current text; remove leading white-spaces, if current-text is empty
                curr_text = curr_text + (
                    token if len(curr_text) > 0 else token.lstrip()
                )

                while (
                    True
                ):  # loop until current-text was not stripped during current iteration

                    # if the goal was cancelled, return immediately
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return Chat.Result()

                    # check if the collected text is visual-code
                    code_visual_match = code_visual_pattern.search(curr_text)
                    if code_visual_match is not None:
                        # extract the visual-code by removing the opening + closing tag and store it as previous text
                        code_visual = code_visual_match.group(1)
                        prev_text = code_visual
                        prev_text_type = Chat.Goal.TEXT_TYPE_CODE_VISUAL
                        # create a chat message from the visual-code, including opening and closing tags
                        chat_message_text = code_visual_match.group(0)
                        if message_id is not None:
                            message_id = self.create_chat_message(chat_id, chat_message_text, False)
                        else:
                            self.update_chat_message(chat_id, message_id, chat_message_text)
                        # self.executor.create_task( #############################
                        #     self.create_chat_message,
                        #     chat_id,
                        #     chat_message_text,
                        #     False,
                        #     bool_update_chat_message,
                        # )
                        # bool_update_chat_message = True
                        # strip the current text
                        curr_text = curr_text[code_visual_match.end() :].rstrip()
                        continue

                    # check if collected text is a sentence
                    sentence_match = sentence_pattern.search(curr_text)
                    if sentence_match is not None:
                        # extract the visual-code by removing the opening + closing tag and store it as previous text
                        sentence = sentence_match.group(1) + (
                            sentence_match.group(3)
                            if sentence_match.group(3) is not None
                            else ""
                        )
                        prev_text = sentence
                        prev_text_type = Chat.Goal.TEXT_TYPE_SENTENCE
                        # create a chat message from the visual-code, including opening and closing tags
                        chat_message_text = sentence
                        if message_id is not None:
                            message_id = self.create_chat_message(chat_id, chat_message_text, False)
                        else:
                            self.update_chat_message(chat_id, message_id, chat_message_text)
                        # self.executor.create_task(
                        #     self.create_chat_message, #################################
                        #     chat_id,
                        #     chat_message_text,
                        #     False,
                        #     bool_update_chat_message,
                        # )
                        # bool_update_chat_message = True
                        # strip the current text
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

        # return the rest of the received text, that has not been forwarded as feedback
        goal_handle.succeed()

        # return the result
        result = Chat.Result()
        if prev_text is None:
            result.text = curr_text
            result.text_type = Chat.Goal.TEXT_TYPE_SENTENCE
        else:
            result.text = prev_text
            result.text_type = prev_text_type
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
