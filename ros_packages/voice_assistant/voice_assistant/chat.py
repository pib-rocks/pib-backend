from dataclasses import dataclass
import re
from threading import Lock
from typing import Callable, Iterable, Optional, Tuple

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

@dataclass
class ResponseSection:
    """
    represents one section of the asssistant's response - either a natural-language sentence or a code visual block (indicated by the 'type' field)

    for a natural-language sentence, the 'core' field represents the actual sentence and the 'total' field contains possible leading/trailing whitespaces
    e.g.: core = "this is a sentence.", total = "   this is a sentence.    "

    for a visual-code-block, the 'core' field represents the actual code, and the 'total' field contains additionally the '<pib-program></pib-program>'
    tags + possible leading/trailing whitespaces, 
    e.g: core = "print('hello world')", total = " <pib-program>print('hello world')</pib-program>  "

    The 'final' field indicates, whether this is the final section of the assistatnt's response 
    """
    core: str
    total: str
    type: bytes
    final: bool

### regex for matching response-sections ###

# matches a visual-code block, that is not the final section of the response
INTERMEDIATE_CODE_VISUAL = r"^(\\s*)<pib-program>(.*?)</pib-program>(?=(\\s*)(\\S))"
# the group of a match that represents the core of the section
INTERMEDIATE_CODE_VISUAL_CORE = 2
# the group of a match that represents the total section
INTERMEDIATE_CODE_VISUAL_TOTAL = 0

# matches a natural-language-sentence that is not the final section of the response
INTERMEDIATE_SENTENCE = r"(\\s*)(((.*?)([^\d | ^A-Z][\.|!|\?|:])(?=(\\s*)(\\S)))|((.*)(\\S)(?=(\\s*)<pib-program>)))"
# the group of a match that represents the core of the section
INTERMEDIATE_SENTENCE_CORE = 2
# the group of a match that represents the total section
INTERMEDIATE_SENTENCE_TOTAL = 0

# matches the final visual-code block
FINAL_CODE_VISUAL = r"^(\\s*)<pib-program>(.*?)</pib-program>(\\s*)"
# the group of a match that represents the core of the section
FINAL_CODE_VISUAL_CORE = 2
# the group of a match that represents the total section
FINAL_CODE_VISUAL_TOTAL = 0

# matches the final natural-language sentence
FINAL_SENTENCE = r"^(\\s*)(.*?)(\\s*)$"
# the group of a match that represents the core of the section
FINAL_SENTENCE_CORE = 2
# the group of a match that represents the total section
FINAL_SENTENCE_TOTAL = 0

### constants representing response-section types ###

SENTENCE_TYPE = Chat.Goal.TEXT_TYPE_SENTENCE
CODE_VISUAL_TYPE = Chat.Goal.TEXT_TYPE_CODE_VISUAL


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

            # for tracking if a message is an update or a new message
            message_id: Optional[str] = None

            # transform assistant response from sequence of tokens to sequence of response-sections
            response_sections = self.tokens_to_sections(tokens, goal_handle.canceled)

            for section in response_sections:

                if message_id is None:
                    message_id = self.create_chat_message(chat_id, section.total, False)
                else:
                    self.update_chat_message(chat_id, message_id, section.total)
                if section.final:
                    result = Chat.Result()
                    result.text = section.core
                    result.text_type = section.type
                    goal_handle.succeed()
                    return result
                else:
                    feedback = Chat.Feedback()
                    feedback.text = section.core
                    feedback.text_type = section.type
                    goal_handle.publish_feedback(feedback)

        except Exception as e:
            self.get_logger().error(f"failed to send request to public-api: {e}")
            goal_handle.abort()
            return Chat.Result()

        # we only end up here, if the goal was cancelled
        return Chat.Result()

    def tokens_to_sections(self, tokens: Iterable[str], is_cancelled: Callable[[], bool]) -> Iterable[ResponseSection]:
        """
        transforms an assistant-response received in form of a sequence of tokens, into a sequence of response-sections.
        stops prematurely, if 'is_cancelled()' yields 'True' at some point
        """

        sentence_pattern = re.compile(INTERMEDIATE_SENTENCE, re.DOTALL)
        code_visual_pattern = re.compile(INTERMEDIATE_CODE_VISUAL, re.DOTALL)
        final_code_visual_pattern = re.compile(FINAL_CODE_VISUAL, re.DOTALL)
        final_sentence_pattern = re.compile(FINAL_SENTENCE, re.DOTALL)

        response = ""

        for token in tokens:
            if is_cancelled():
                self.get_logger().info("response-section generation cancelled")
                return
            response += token
            while True:
                sentence_match = sentence_pattern.search(response)
                if sentence_match is not None:
                    core = sentence_match[INTERMEDIATE_SENTENCE_CORE]
                    total = sentence_match[INTERMEDIATE_SENTENCE_TOTAL]
                    text = text[len(total):]
                    yield ResponseSection(core, total, SENTENCE_TYPE, False)
                    continue
                code_visual_match = code_visual_pattern.search(response)
                if code_visual_match is not None:
                    core = code_visual_match[INTERMEDIATE_CODE_VISUAL_CORE]
                    total = code_visual_match[INTERMEDIATE_CODE_VISUAL_TOTAL]
                    text = text[len(total):]
                    yield ResponseSection(core, total, CODE_VISUAL_TYPE, False)
                    continue
                break
        
        final_code_visual_match = final_code_visual_pattern.search(response)
        if final_code_visual_match is not None:
            core = final_code_visual_match[FINAL_CODE_VISUAL_CORE]
            total = final_code_visual_match[FINAL_CODE_VISUAL_TOTAL]
            yield ResponseSection(core, total, CODE_VISUAL_TYPE, True)
        else:
            final_sentence_match = final_sentence_pattern.search(response)
            core = final_sentence_match[FINAL_SENTENCE_CORE]
            total = final_sentence_match[FINAL_SENTENCE_TOTAL]
            yield ResponseSection(core, total, SENTENCE_TYPE, True)

            

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
