import re
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.publisher import Publisher

from datatypes.msg import ChatMessage
from datatypes.action import Chat

from public_api_client import public_voice_client
from pib_api_client import voice_assistant_client


CODE_DESCRIPTION_PREFIX = 'You answer as a humanoid robot, that can be programmed via a json-api. If you want the robot to make a movement, include this part in <pib-program></pib-program> tags. This is an example of how the json-code should look like: {"blocks":{"languageVersion":0,"blocks":[{"type":"controls_repeat_ext","inputs":{"TIMES":{"shadow":{"type":"math_number","fields":{"NUM":4}}},"DO":{"block":{"type":"move_motor","fields":{"MOTORNAME":"TURN_HEAD","MODE":"ABSOLUTE"},"inputs":{"POSITION":{"block":{"type":"math_number","fields":{"NUM":5000}}}},"next":{"block":{"type":"sleep_for_seconds","fields":{"SECONDS":0.4},"next":{"block":{"type":"move_motor","fields":{"MOTORNAME":"TURN_HEAD","MODE":"ABSOLUTE"},"inputs":{"POSITION":{"block":{"type":"math_number","fields":{"NUM":-5000}}}},"next":{"block":{"type":"sleep_for_seconds","fields":{"SECONDS":0.4}}}}}}}}}},"next":{"block":{"type":"move_motor","fields":{"MOTORNAME":"INDEX_LEFT_STRETCH","MODE":"ABSOLUTE"},"inputs":{"POSITION":{"block":{"type":"math_number","fields":{"NUM":9000}}}}}}}]}}. The code provided in the previous example will cause you to shake your head 4 times from left to right with small pauses in between movements and finally you will stretch your index finger once. In the "MOTORNAME" field you can specify one of the following motor-names: "THUMB_LEFT_OPPOSITION", "THUMB_LEFT_STRETCH", "INDEX_LEFT_STRETCH", "MIDDLE_LEFT_STRETCH", "RING_LEFT_STRETCH", "PINKY_LEFT_STRETCH", "ALL_FINGERS_LEFT_STRETCH", "THUMB_RIGHT_OPPOSITION", "THUMB_RIGHT_STRETCH", "INDEX_RIGHT_STRETCH", "MIDDLE_RIGHT_STRETCH", "RING_RIGHT_STRETCH", "PINKY_RIGHT_STRETCH", "ALL_FINGERS_RIGHT_STRETCH", "UPPER_ARM_LEFT_ROTATION", "ELBOW_LEFT", "LOWER_ARM_LEFT_ROTATION", "WRIST_LEFT", "SHOULDER_VERTICAL_LEFT", "SHOULDER_HORIZONTAL_LEFT", "UPPER_ARM_RIGHT_ROTATION", "ELBOW_RIGHT", "LOWER_ARM_RIGHT_ROTATION", "WRIST_RIGHT", "SHOULDER_VERTICAL_RIGHT", "SHOULDER_HORIZONTAL_RIGHT", "TILT_FORWARD_HEAD", "TURN_HEAD". Each motor\'s position can be set to a value between -9000 and +9000.'


class ChatNode(Node):

    def __init__(self):

        super().__init__('chat')

        # server for communicating with an llm via tryb's public-api
        # In the goal, a client specifies some text that will be sent as input to the llm, as well as the 
        # description of the personality. The server then forwards the llm output to the client at the
        # granularity of sentences. Intermediate sentences, are forwared in form of feedback. The final
        # sentence is forwarded as the result of the goal
        self.chat_server = ActionServer(
            self, 
            Chat, 
            'chat', 
            execute_callback=self.chat,
            cancel_callback=(lambda _ : CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup())
        
        # Publisher for ChatMessages
        self.chat_message_publisher: Publisher = self.create_publisher(
            ChatMessage, 
            "chat_messages",
            10)
        
        # lock that should be aquired, whenever accessing 'public_voice_client'
        self.public_voice_client_lock = Lock()
        # lock that should be aquired, whenever accessing 'voice_assistant_client'
        self.voice_assistant_client_lock = Lock()

        self.get_logger().info('Now running CHAT')



    def create_chat_message(self, chat_id: str, text: str, is_user: bool) -> None:
        """writes a new chat-message to the db, and publishes it to the 'chat_messages'-topic"""

        if text == "": return

        with self.voice_assistant_client_lock:
            successful, chat_message = voice_assistant_client.create_chat_message(chat_id, text, is_user)
        if not successful: 
            self.get_logger().error(f"unable to create chat message: {(chat_id, text, is_user)}")
            return

        chat_message_ros = ChatMessage()
        chat_message_ros.chat_id = chat_id
        chat_message_ros.content = chat_message.content
        chat_message_ros.is_user = chat_message.is_user
        chat_message_ros.message_id = chat_message.message_id
        chat_message_ros.timestamp = chat_message.timestamp

        self.chat_message_publisher.publish(chat_message_ros)



    def chat(self, goal_handle: ServerGoalHandle):

            # unpack request data            
            request: Chat.Goal = goal_handle.request
            chat_id: str = request.chat_id
            content: str = request.text
            generate_code: bool = request.generate_code

            # get the personality that is associated with the request chat_id from the pib-api
            with self.voice_assistant_client_lock: 
                successful, personality = voice_assistant_client.get_personality_from_chat(chat_id)
            if not successful:
                self.get_logger().info(f"no personality found for id {chat_id}")
                goal_handle.abort()
                return Chat.Result() # TODO
            
            # create the user message
            self.executor.create_task(self.create_chat_message, chat_id, content, True)

            # receive an iterable of tokens from the public-api
            description = personality.description if personality.description is not None else "Du bist pib, ein humanoider Roboter."
            if generate_code: description = CODE_DESCRIPTION_PREFIX + description
            print(description)
            with self.public_voice_client_lock:
                tokens = public_voice_client.chat_completion(content, description)

            # regex for indentifying sentences
            sentence_pattern = re.compile(r"^(?!<pib-program>)(.*?)(([^\d | ^A-Z][\.|!|\?|:])|<pib-program>)", re.DOTALL)
            # regex-pattern for indentifying visual-code blocks
            code_visual_pattern = re.compile(r"^<pib-program>(.*?)</pib-program>", re.DOTALL)

            # the text that was currently collected by chaining together tokens
            # at any given point in time, this string must not contain any leading whitespaces!
            curr_text: str = ""
            # previously collected text, that is waiting to be published as feedback
            prev_text: str | None = None

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
                curr_text = curr_text + (token if len(curr_text) > 0 else token.lstrip())

                while True: # loop until current-text was not stripped during current iteration

                    # if the goal was cancelled, return immediately
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return Chat.Result() # TODO
                    
                    # check if the collected text is visual-code
                    code_visual_match = code_visual_pattern.search(curr_text)
                    if code_visual_match is not None:
                        # extract the visual-code by removing the opening + closing tag and store it as previous text
                        code_visual = code_visual_match.group(1)
                        prev_text = code_visual
                        prev_text_type = Chat.Goal.TEXT_TYPE_CODE_VISUAL
                        # create a chat message from the visual-code, including opening and closing tags
                        chat_message_text = code_visual_match.group(0)
                        self.executor.create_task(self.create_chat_message, chat_id, chat_message_text, False)
                        # strip the current text
                        curr_text = curr_text[code_visual_match.end():].rstrip()
                        continue

                    # check if collected text is a sentence
                    sentence_match = sentence_pattern.search(curr_text)
                    if sentence_match is not None:
                        # extract the visual-code by removing the opening + closing tag and store it as previous text
                        sentence = sentence_match.group(1) + (sentence_match.group(3) if sentence_match.group(3) is not None else "")
                        prev_text = sentence
                        prev_text_type = Chat.Goal.TEXT_TYPE_SENTENCE
                        # create a chat message from the visual-code, including opening and closing tags
                        chat_message_text = sentence
                        self.executor.create_task(self.create_chat_message, chat_id, chat_message_text, False)
                        # strip the current text
                        curr_text = curr_text[sentence_match.end(3 if sentence_match.group(3) is not None else 1):].rstrip()
                        continue
                        
                    break

            # create chat-message for remaining input
            if (len(curr_text) > 0):
                self.executor.create_task(self.create_chat_message, chat_id, curr_text, False)

            # return the rest of the received text, that has not been forwarded as feedback
            goal_handle.succeed()

            # return the restult
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
