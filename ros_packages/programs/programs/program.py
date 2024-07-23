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

from threading import Lock, Thread
from queue import Queue
from subprocess import Popen, PIPE
from typing import IO, Tuple
import time
import tempfile

from datatypes.action import RunProgram
from datatypes.msg import ProgramOutputLine, ProgramInput

import pib_blockly_client

PYTHON_BINARY: str = os.getenv(
    "PYTHON_BINARY",
    "/home/pib/ros_working_dir/src/programs/user_program_env/bin/python3",
)
UNBUFFERED_OUTPUT_FLAG: str = "-u"
PROGRAM_DIR: str = os.getenv("PROGRAM_DIR", "/home/pib/cerebra_programs")

ACTION_LOOP_WAITING_PERIOD_SECONDS: float = 0.05


class ProgramNode(Node):

    def __init__(self) -> None:

        super().__init__("program")

        # Quality-of-Service profile for feedback
        feedback_profile = qos.QoSProfile(
            history=qos.HistoryPolicy.KEEP_ALL,
            reliability=qos.ReliabilityPolicy.RELIABLE,
            durability=qos.DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=100),
        )

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

        self.program_input_subscription = self.create_subscription(
            ProgramInput,
            "program_input",
            self.program_input_callback,
            qos.QoSProfile(history=qos.HistoryPolicy.KEEP_ALL),
        )

        # perform cleanup periodically
        self.cleanup_timer = self.create_timer(30, self.cleanup)

        # the mpid is used to identify a running instance of a program
        self.next_mpid = 0
        self.next_mpid_lock = Lock()

        # access processes via their mpid
        self.mpid_to_process: dict[int, Popen] = {}
        self.process_lock = Lock()

        self.get_logger().info("Now Running PROGRAM")

    def cleanup(self) -> None:
        with self.process_lock:
            self.mpid_to_process = {
                mpid: process
                for mpid, process in self.mpid_to_process.items()
                if process.poll() is None
            }

    def program_input_callback(self, program_input: ProgramInput) -> None:
        self.get_logger().info(f"received input: {program_input}")
        with self.process_lock:
            try:
                process = self.mpid_to_process[program_input.mpid]
                process.stdin.write(program_input.input + "\n")
            except KeyError:
                self.get_logger().warn(
                    f"attempted to provide input to process {program_input.mpid}, but no such process was found"
                )
            except BrokenPipeError:
                self.get_logger().warn(
                    f"attempted to provide input to process {program_input.mpid}, but it seems that the process has already terminated (broken pipe)"
                )
            except Exception as e:
                self.get_logger().error(
                    f"unexpected error occured while trying to provide input to process: {e}"
                )

    def run_program_callback(self, goal_handle: ServerGoalHandle) -> RunProgram.Result:

        # digest the code-source and create a path to the resulting python-code file that should be executed
        request: RunProgram.Goal = goal_handle.request
        if request.source_type == RunProgram.Goal.SOURCE_PROGRAM_NUMBER:
            program_number = request.source
            self.get_logger().info(
                f"received request to execute program of number {request.source}."
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

        with self.next_mpid_lock:
            mpid = self.next_mpid
            self.next_mpid += 1

        # publish only the mpid as initial feedback
        init_feedback = RunProgram.Feedback()
        init_feedback.mpid = mpid
        goal_handle.publish_feedback(init_feedback)

        try:
            self.get_logger().info("starting execution of program...")

            run_python_program = [
                PYTHON_BINARY,
                UNBUFFERED_OUTPUT_FLAG,
                code_python_file_path,
            ]
            process = Popen(
                run_python_program,
                stdout=PIPE,
                stderr=PIPE,
                stdin=PIPE,
                universal_newlines=True,
                bufsize=1,
            )
            output_queue: Queue[Tuple[str, bool]] = Queue()

            def forward_io_to_connection(output: IO[str], is_stderr: bool) -> None:
                for line in output:
                    output_queue.put((line[:-1], is_stderr))

            forward_stdout = Thread(
                target=forward_io_to_connection, args=(process.stdout, False)
            )
            forward_stderr = Thread(
                target=forward_io_to_connection, args=(process.stderr, True)
            )

            with self.process_lock:
                self.mpid_to_process[mpid] = process

            forward_stdout.start()
            forward_stderr.start()

            # loop until either cancellation of goal is requested, or python-program terminated
            while True:

                # if cancellation of the goal if requested, terminate the process
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    process.terminate()
                    return RunProgram.Result(exit_code=2)

                # collect output of the user-program
                output_lines: list[ProgramOutputLine] = []
                while not output_queue.empty():
                    line, is_stderr = output_queue.get()
                    output_line = ProgramOutputLine()
                    output_line.content = line
                    output_line.is_stderr = is_stderr
                    output_lines.append(output_line)

                # if at least one line of stdout/stderr output was collected, send it as feedback
                if len(output_lines) > 0:
                    feedback = RunProgram.Feedback()
                    feedback.output_lines = output_lines
                    feedback.mpid = mpid
                    goal_handle.publish_feedback(feedback)

                return_code = process.poll()
                if return_code is not None:
                    if forward_stdout.is_alive() or forward_stderr.is_alive():
                        forward_stdout.join()
                        forward_stderr.join()
                        continue
                    else:
                        goal_handle.succeed()
                        return RunProgram.Result(exit_code=return_code)

                time.sleep(ACTION_LOOP_WAITING_PERIOD_SECONDS)

        # if the code-source is visual-code, a temp. file was created, which must now be deleted
        finally:
            self.get_logger().info("execution finished, cleaning up...")
            if request.source_type == RunProgram.Goal.SOURCE_CODE_VISUAL:
                os.remove(code_python_file_path)


def main(args=None) -> None:
    rclpy.init()
    program_node = ProgramNode()
    executor = MultiThreadedExecutor(8)
    executor.add_node(program_node)
    executor.spin()
    program_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
