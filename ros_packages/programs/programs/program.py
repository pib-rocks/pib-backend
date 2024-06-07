import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import CancelResponse
from rclpy import qos
from rclpy.duration import Duration

from multiprocessing import Process, Pipe
from threading import Lock, Thread
from subprocess import Popen, PIPE
from multiprocessing.connection import Connection
from typing import IO
import time
import tempfile

from datatypes.action import RunProgram
from datatypes.msg import ProgramOutputLine

import pib_blockly_client

PYTHON_BINARY: str = os.getenv(
    "PYTHON_BINARY",
    "/home/pib/ros_working_dir/src/programs/user_program_env/bin/python3",
)
UNBUFFERED_OUTPUT_FLAG: str = "-u"
PROGRAM_DIR: str = os.getenv("PROGRAM_DIR", "/home/pib/cerebra_programs")

MAIN_LOOP_WAITING_PERIOD_SECONDS: float = 0.1
ACTION_LOOP_WAITING_PERIOD_SECONDS: float = 0.05


# request to start execution of user-program, sent from ros-process to main-process
class StartRequest:
    def __init__(self, goal_id: bytes, code_python_file_path: str) -> None:
        self.goal_id = goal_id
        self.code_python_file_path = code_python_file_path


# response sent from main-process to ros-process, after ros-process sent a 'StartRequest'
class StartResponse:
    def __init__(self, successful: bool, connection: Connection) -> None:
        self.successful = successful
        self.connection = connection


# request to stop execution of user-program, sent from ros-process to main-process
class StopRequest:
    def __init__(self, goal_id: bytes) -> None:
        self.goal_id = goal_id


# response sent from main-process to ros-process, after ros-process sent a 'StopRequest'
class StopResponse:
    def __init__(self, successful: bool) -> None:
        self.successful = successful


# one line of stdout/stderr output of a running user-program, sent from a host-process to the ros-process
class OutputLine:
    def __init__(self, content: str, is_stderr: bool) -> None:
        self.content = content
        self.is_stderr = is_stderr


# exit-code of a user-program, sent from a host-process to the ros-process
class ExitCode:
    def __init__(self, code: int) -> None:
        self.code = code


class ProgramNode(Node):

    def __init__(self, request_sender: Connection) -> None:

        super().__init__("program")

        # Quality-of-Service profile for feedback
        feedback_profile = qos.QoSProfile(
            history=qos.HistoryPolicy.KEEP_ALL,
            reliability=qos.ReliabilityPolicy.RELIABLE,
            durability=qos.DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=100),
        )

        # used for requesting the main prcess to start/stop a python program
        self.request_sender: Connection = request_sender
        self.request_sender_lock: type[Lock] = Lock()

        # server for running local python-programs generated with blockly
        self.run_program_server = ActionServer(
            self,
            RunProgram,
            "run_program",
            self.run_program_callback,
            cancel_callback=(lambda _: CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup(),
            feedback_pub_qos_profile=feedback_profile,
        )

        self.get_logger().info("Now Running PROGRAM")

    def run_program_callback(self, goal_handle: ServerGoalHandle) -> RunProgram.Result:

        # convert goal id to bytes
        goal_id: bytes = bytes([int(num) for num in goal_handle.goal_id.uuid])

        # digest the code-source and create a path to the resulting python-code file that should be executed
        request: RunProgram.Goal = goal_handle.request
        if request.source_type == RunProgram.Goal.SOURCE_PROGRAM_NUMBER:
            program_number = request.source
            self.get_logger().info(
                "received request to execute program of number {source}."
            )
            code_python_file_path = f"{PROGRAM_DIR}/{program_number}.py"
        elif request.source_type == RunProgram.Goal.SOURCE_CODE_VISUAL:
            self.get_logger().info("received request to execute some visual-code.")
            code_visual = request.source
            successful, code_python = pib_blockly_client.code_visual_to_python(
                code_visual
            )
            if not successful:
                goal_handle.abort()
                return RunProgram.Result(exit_code=2)
            self.get_logger().info(
                "visual-code was successfully compiled to python-code."
            )
            fd, code_python_file_path = tempfile.mkstemp()
            with os.fdopen(fd, "w") as file:
                file.write(code_python)
        else:
            self.get_logger().info(
                f"received unexpected source type: {request.source_type}."
            )
            goal_handle.abort()
            return RunProgram.Result(exit_code=2)

        try:
            self.get_logger().info("starting execution of program...")
            # send request to main-process to start the program
            start_request = StartRequest(goal_id, code_python_file_path)
            with self.request_sender_lock:
                self.request_sender.send(start_request)
                response: StartResponse = self.request_sender.recv()
                if not response.successful:
                    goal_handle.abort()
                    return RunProgram.Result(exit_code=2)
                output_receiver: Connection = response.connection

            while (
                True
            ):  # loop until either cancellation of goal is requested, or python-program terminated

                # if cancellation of the goal if requested, send termination-request to main-process
                if goal_handle.is_cancel_requested:
                    stop_request = StopRequest(goal_id)
                    with self.request_sender_lock:
                        self.request_sender.send(stop_request)
                        response: StopResponse = self.request_sender.recv()
                        if response.successful:
                            goal_handle.canceled()
                            return RunProgram.Result(exit_code=2)

                # collect output of the user-program
                output_lines: list[ProgramOutputLine] = []
                exit_code = None
                while output_receiver.poll():
                    output = output_receiver.recv()
                    if isinstance(output, OutputLine):
                        output_line = ProgramOutputLine(
                            content=output.content, is_stderr=output.is_stderr
                        )
                        output_lines.append(output_line)
                    elif isinstance(output, ExitCode):
                        exit_code = output.code
                    else:
                        raise RuntimeError(
                            f"ros-process received unexpected output-type from host-process: {type(output)}'"
                        )

                # if at least one line of stdout/stderr output was collected, send it as feedback
                if len(output_lines) > 0:
                    goal_handle.publish_feedback(
                        RunProgram.Feedback(output_lines=output_lines)
                    )

                # if an exit-code was collected, return it as the result of the goal
                if exit_code is not None:
                    goal_handle.succeed()
                    return RunProgram.Result(exit_code=exit_code)

                time.sleep(ACTION_LOOP_WAITING_PERIOD_SECONDS)

        # if the code-source is viual-code, a temp. file was created, which must now be deleted
        finally:
            self.get_logger().info("execution finished, cleaning up...")
            if request.source_type == RunProgram.Goal.SOURCE_CODE_VISUAL:
                os.remove(code_python_file_path)


def main_loop(request_receiver: Connection) -> None:

    goal_id_to_host: dict[bytes, Process] = {}

    while True:

        # handle all requests that were received from the ros-process during waiting period
        while request_receiver.poll():
            request = request_receiver.recv()
            if isinstance(request, StartRequest):
                output_sender, output_receiver = Pipe()
                request_receiver.send(StartResponse(True, output_receiver))
                host_process = Process(
                    target=run_program,
                    args=(request.code_python_file_path, output_sender),
                )
                goal_id_to_host[request.goal_id] = host_process
                host_process.start()
            elif isinstance(request, StopRequest):
                try:
                    host_process = goal_id_to_host.pop(request.goal_id)
                except KeyError:
                    request_receiver.send(StopResponse(False))
                    continue
                host_process.terminate()
                request_receiver.send(StopResponse(True))
            else:
                raise RuntimeError(
                    f"main-process received unexpected request-type from ros-process: {type(request)}'"
                )

        # filter out host-processes that have already terminated
        goal_id_to_host = {
            id: host for id, host in goal_id_to_host.items() if host.is_alive()
        }

        time.sleep(MAIN_LOOP_WAITING_PERIOD_SECONDS)


def ros_target(request_sender: Connection) -> None:

    rclpy.init()
    program_node = ProgramNode(request_sender)
    executor = MultiThreadedExecutor(4)
    executor.add_node(program_node)
    executor.spin()
    program_node.destroy_node()
    rclpy.shutdown()


def run_program(code_python_file_path: str, output_sender: Connection) -> None:

    run_python_program = [PYTHON_BINARY, UNBUFFERED_OUTPUT_FLAG, code_python_file_path]
    with Popen(
        run_python_program, stdout=PIPE, stderr=PIPE, universal_newlines=True, bufsize=1
    ) as popen:

        output_sender_lock = Lock()

        def forward_io_to_connection(output: IO[str], is_stderr: bool) -> None:
            for line in output:
                with output_sender_lock:
                    output_sender.send(OutputLine(line[:-1], is_stderr))

        forward_stdout = Thread(
            target=forward_io_to_connection, args=(popen.stdout, False)
        )
        forward_stderr = Thread(
            target=forward_io_to_connection, args=(popen.stderr, True)
        )

        forward_stdout.start()
        forward_stderr.start()

        return_code = popen.wait()

        forward_stdout.join()
        forward_stderr.join()

        output_sender.send(ExitCode(return_code))


def main(args=None) -> None:
    request_sender, request_receiver = Pipe()
    ros_process = Process(target=ros_target, args=(request_sender,))
    ros_process.start()

    main_loop(request_receiver)


if __name__ == "__main__":
    main()
