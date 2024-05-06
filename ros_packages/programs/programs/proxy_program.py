import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import  ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.action.client import GoalStatus
from rclpy import qos

from datatypes.action import RunProgram
from datatypes.msg import ProxyRunProgramFeedback, ProxyRunProgramResult, ProxyRunProgramStatus
from datatypes.srv import ProxyRunProgramStart, ProxyRunProgramStop
from rclpy.duration import Duration

from rclpy.task import Future
from uuid import uuid4


############################################################################################################
# this node acts as a proxy for the actual program-node, specifically for since 'roslibjthe 'run-program' action
# that the program-node provides. The proxy node is required, s' does not support action-
# communication with ros2 (only with ros). At the time of writing (02-02-2024), this functionality is currently in 
# development and already implemented on the 'develop' branch of the roslibjs-github-repo, 
# though no offical release is available yet. If such a release is available and becomes integrated into 
# the cerebra-frontend, this proxy may be removed and the frontend may directly talk to the actual program-node
############################################################################################################


# these states indicate, that the associated action-goal has terminated
TERMINAL_GOAL_STATES = { GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED }



class ProxyProgramNode(Node):

    def __init__(self) -> None:

        super().__init__('proxy_program')

        # Quality-of-Service profile for feedback
        feedback_profile = qos.QoSProfile(
            history=qos.HistoryPolicy.KEEP_ALL,
            reliability=qos.ReliabilityPolicy.RELIABLE,
            durability=qos.DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=100))  

        # maps a proxy_goal_id to its associated goal_handle and its previous status (in order to detect, if the status
        # of the goal has changed after it was last checked)
        self.id_to_goal: dict[str, tuple[ClientGoalHandle, int]] = {}
        
        # each topic is corresponding to one of the three output types that actions provide: feedback, result and status 
        self.feedback_publisher = self.create_publisher(ProxyRunProgramFeedback, 'proxy_run_program_feedback', feedback_profile)
        self.result_publisher = self.create_publisher(ProxyRunProgramResult, 'proxy_run_program_result', 10)
        self.status_publisher = self.create_publisher(ProxyRunProgramStatus, 'proxy_run_program_status', 10)

        # one service for making a request to the action (start), and one for cancelling a goal (stop)
        self.start_program = self.create_service(ProxyRunProgramStart, 'proxy_run_program_start', self.start_program_callback)
        self.stop_program = self.create_service(ProxyRunProgramStop, 'proxy_run_program_stop', self.stop_program_callback)

        # check periodically for the statuses of the current goals and (in case somethong has changed) publish the status
        self.status_loop_timer = self.create_timer(0.1, self.status_loop_callback)


        # client for the 'RunProgram'-Action, that this node acts as a proxy for
        self.run_program_client = ActionClient(self, RunProgram, 'run_program', feedback_sub_qos_profile=feedback_profile)
        self.run_program_client.wait_for_server()

        self.get_logger().info('Now Running PROXY PROGRAM')



    def start_program_callback(self, request: ProxyRunProgramStart.Request, response: ProxyRunProgramStart.Response) -> ProxyRunProgramStart.Response:

        # the uuid is used to identify an action-goal and is sent with every topic message, so that they 
        # can be filtered  for the messages, that actually belong to the goal, that the client has initiated
        proxy_goal_id = str(uuid4())

        # when receiving feedback from the action-goal, publish it to the feedback-topic together with the id
        def forward_feedback_to_publisher(feedback_message) -> None:
            feedback: RunProgram.Feedback = feedback_message.feedback
            proxy_feedback = ProxyRunProgramFeedback(
                proxy_goal_id=proxy_goal_id, 
                output_lines=feedback.output_lines)
            self.feedback_publisher.publish(proxy_feedback)

        # when receiving the result of the action-goal, publish it to the resul-topic together with the id
        def receive_result(result_future: Future) -> None:
            result: RunProgram.Result = result_future.result().result
            self.result_publisher.publish(ProxyRunProgramResult(
                exit_code=result.exit_code, 
                proxy_goal_id=proxy_goal_id))

        def receive_goal_handle(goal_handle_future: Future) -> None:
            goal_handle: ClientGoalHandle = goal_handle_future.result()
            self.id_to_goal[proxy_goal_id] = (goal_handle, -1)
            result_future: Future = goal_handle.get_result_async()
            result_future.add_done_callback(receive_result)

        goal = RunProgram.Goal()
        goal.source = request.program_number
        goal.source_type = RunProgram.Goal.SOURCE_PROGRAM_NUMBER
        future: Future = self.run_program_client.send_goal_async(goal, forward_feedback_to_publisher)
        future.add_done_callback(receive_goal_handle)

        response.proxy_goal_id = proxy_goal_id
        return response



    def stop_program_callback(self, request: ProxyRunProgramStop.Request, response: ProxyRunProgramStop.Response) -> ProxyRunProgramStop.Response:

        try: goal_handle: ClientGoalHandle = self.id_to_goal[request.proxy_goal_id][0]
        except: return response
        goal_handle.cancel_goal_async()
        return response



    def status_loop_callback(self) -> None:

        proxy_goal_id: str
        status: int
        goal_handle: ClientGoalHandle

        for proxy_goal_id, tuple in self.id_to_goal.items():
            goal_handle, status = tuple
            next_status = goal_handle.status
            if status != next_status:
                self.status_publisher.publish(ProxyRunProgramStatus(proxy_goal_id=proxy_goal_id, status=next_status))
                self.id_to_goal[proxy_goal_id] = (goal_handle, next_status)

        # remove all goal_handles that have already terminated
        self.id_to_goal = { id : t for id , t in self.id_to_goal.items() if t[1] not in TERMINAL_GOAL_STATES }



def main(args=None) -> None:

    rclpy.init()
    proxy_program_node = ProxyProgramNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(proxy_program_node)
    executor.spin()
    proxy_program_node.destroy_node()
    rclpy.shutdown()
 


if __name__ == "__main__":
    main()