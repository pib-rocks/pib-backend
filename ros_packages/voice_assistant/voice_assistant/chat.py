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

from datatypes.action import Chat

from public_api_client import public_voice_client



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
        
        # the public_voice_client is the only shared resource between threads
        # and should be secured with a lock
        self.public_voice_client_lock = Lock()

        self.get_logger().info('Now running CHAT')



    def chat(self, goal_handle: ServerGoalHandle):
            
            request: Chat.Goal = goal_handle.request

            # receive an iterable of tokens from the public-api
            with self.public_voice_client_lock:
                tokens = public_voice_client.chat_completion(request.text, request.description)

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
                    feedback = Chat.Feedback()
                    feedback.sentence = prev_sentence
                    goal_handle.publish_feedback(feedback)
                    prev_sentence = None
                # check if the current token marks the end of a sentence
                if sentence_boundary.search(curr_sentence):
                    prev_sentence = curr_sentence.strip()
                    curr_sentence = ""
                curr_sentence += token

            goal_handle.succeed()
            # return the rest of the received text, that has not been forwarded as feedback
            return Chat.Result(rest=curr_sentence)    



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
