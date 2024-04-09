import re

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from datatypes.action import Chat

from public_api_client import public_voice_client

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle



class ChatNode(Node):

    def __init__(self):

        super().__init__('chat')

        # server for running local python-programs generated with blockly
        self.chat_server = ActionServer(
            self, 
            Chat, 
            'chat', 
            execute_callback=self.chat,
            cancel_callback=(lambda _ : CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Now running CHAT')



    def chat(self, goal_handle: ServerGoalHandle):
        
            request: Chat.Goal = goal_handle.request
            tokens = public_voice_client.chat_completion(request.text, request.description)

            buffer: str = ""
            sentence: str | None = None
            sentence_boundary = re.compile(r"[^\d | ^A-Z][\.|!|\?|:]")

            for token in tokens:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Chat.Result(rest=buffer)  
                if sentence is not None: 
                    feedback = Chat.Feedback()
                    feedback.sentence = sentence
                    goal_handle.publish_feedback(feedback)
                    sentence = None
                if sentence_boundary.search(buffer):
                    sentence = buffer.strip()
                    buffer = ""
                buffer += token

            goal_handle.succeed()
            return Chat.Result(rest=buffer)    



def main(args=None):

    rclpy.init()
    node = ChatNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
