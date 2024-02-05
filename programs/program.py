import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import CancelResponse

from multiprocessing import Process, Pipe, Lock
from threading import Lock, Thread
from subprocess import Popen, PIPE
from enum import Enum
from multiprocessing.connection import Connection
from typing import IO
import time

from datatypes.action import RunProgram



PYTHON_BINARY: str = '/home/pib/ros_working_dir/src/programs/user_program_env/bin/python3'
UNBUFFERED_OUTPUT_FLAG: str = '-u'
PYTHON_SCRIPT: str = '/home/pib/cerebra_programs/%s.py'

MAIN_LOOP_WAITING_PERIOD_SECONDS = 0.1
ACTION_LOOP_WAITING_PERIOD_SECONDS = 0.05



# a request sent by the action server to the main process, to start/stop a program
class Request:

    class Operation(Enum):
        START = 0
        STOP = 1

    def __init__(self, goal_id: str, operation: Operation, program_number: str = ''):
        self.goal_id = goal_id
        self.operation = operation
        self.program_number = program_number


# a response, sent from the main-process to the ros-process, indicating if the program
# was successfully started/stopped; in case it was started, the provided connection
# can be used to access its output
class Response:

    def __init__(self, successful: bool, connection: Connection | None = None):
        self.successful = successful
        self.connection = connection



# represents output of a process - if type if stdout/stderr, the value should be a line of console output,
# otherwise it should be an integer, representing the exit-code of the process
class ProgramOutput:

    class Type(Enum):
        STDOUT = 0
        STDERR = 1
        EXIT_CODE = 2

    def __init__(self, type: Type, value: str | int):
        self.type = type
        self.value = value



class ProgramNode(Node):

    def __init__(self, request_sender: Connection):

        super().__init__('program')

        # used for requesting the main prcess to start/stop a python program
        self.request_sender: Connection = request_sender
        self.request_sender_lock: type[Lock] = Lock()

        # server for running local python-programs generated with blockly
        self.run_program_server = ActionServer(
            self, 
            RunProgram, 
            'run_program', 
            self.run_program_callback,
            cancel_callback=(lambda _ : CancelResponse.ACCEPT),
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('--- program node started successfully ---')



    def run_program_callback(self, goal_handle: ServerGoalHandle):

        # convert goal id to bytes
        goal_id: bytes = bytes([ int(num) for num in goal_handle.goal_id.uuid ])

        # send request to main-process to start the program
        request = Request(goal_id, Request.Operation.START, goal_handle.request.program_number)
        with self.request_sender_lock:
            self.request_sender.send(request)
            response: Response = self.request_sender.recv()
            if not response.successful:
                goal_handle.abort()
                return RunProgram.Result(exit_code=2)
            output_receiver: Connection = response.connection

        while True: # loop until either cancellation of goal is requested, or python-program terminated

            # if cancellation of the goal if requested, send termination-request to main-process
            if goal_handle.is_cancel_requested:
                request = Request(goal_id, Request.Operation.STOP)
                with self.request_sender_lock:
                    self.request_sender.send(request)
                    response: Response = self.request_sender.recv()
                    if response.successful:
                        goal_handle.canceled()
                        return RunProgram.Result(exit_code=2)

            # forward all program-output to either the action's feedback or result
            while output_receiver.poll():
                output: ProgramOutput = output_receiver.recv()
                if output.type == ProgramOutput.Type.EXIT_CODE:
                    goal_handle.succeed()
                    return RunProgram.Result(exit_code=output.value)
                else:
                    feedback = RunProgram.Feedback()
                    feedback.output_line = output.value
                    feedback.is_stderr = (output.type == ProgramOutput.Type.STDERR)
                    goal_handle.publish_feedback(feedback)

            time.sleep(ACTION_LOOP_WAITING_PERIOD_SECONDS)



def main_loop(request_receiver: Connection):
    
    goal_id_to_host: dict[bytes, Process] = {}

    while True:

        # handle all requests that were received from the ros-process during waiting period
        while request_receiver.poll():
            request: Request = request_receiver.recv()
            if request.operation == Request.Operation.START:
                output_sender, output_receiver = Pipe()
                request_receiver.send(Response(True, output_receiver))
                host_process = Process(target=run_program, args=(request.program_number, output_sender))
                goal_id_to_host[request.goal_id] = host_process
                host_process.start()
            elif request.operation == Request.Operation.STOP:
                try: 
                    host_process = goal_id_to_host.pop(request.goal_id)
                except KeyError: 
                    request_receiver.send(Response(False))
                    continue
                host_process.terminate()
                request_receiver.send(Response(True))
            else: raise Exception(f'unexpected operation in request: {request.operation}')
        
        # filter out host-processes that have already terminated
        goal_id_to_host = { id : host for id , host in goal_id_to_host.items() if host.is_alive() }

        time.sleep(MAIN_LOOP_WAITING_PERIOD_SECONDS)



def ros_target(request_sender: Connection) -> None:

    rclpy.init()
    program_node = ProgramNode(request_sender)
    executor = MultiThreadedExecutor(4)
    executor.add_node(program_node)
    executor.spin()
    program_node.destroy_node()
    rclpy.shutdown()



def run_program(program_number: str, output_sender: Connection) -> None:

    run_python_program = [PYTHON_BINARY, UNBUFFERED_OUTPUT_FLAG, PYTHON_SCRIPT % program_number]
    with Popen(run_python_program, stdout=PIPE, stderr=PIPE, universal_newlines=True, bufsize=1) as popen:

        output_sender_lock = Lock()

        def forward_io_to_connection(output: IO[str], output_type: ProgramOutput.Type) -> None:
            for line in output: 
                with output_sender_lock: 
                    output_sender.send(ProgramOutput(output_type, line[:-1]))

        forward_stdout = Thread(target=forward_io_to_connection, args=(popen.stdout, ProgramOutput.Type.STDOUT))
        forward_stderr = Thread(target=forward_io_to_connection, args=(popen.stderr, ProgramOutput.Type.STDERR))

        forward_stdout.start()
        forward_stderr.start()

        return_code = popen.wait()

        forward_stdout.join()
        forward_stderr.join()

        output = ProgramOutput(ProgramOutput.Type.EXIT_CODE, return_code)
        output_sender.send(output)



def main(args=None):

    request_sender, request_receiver = Pipe()
    ros_process = Process(target=ros_target, args=(request_sender,))
    ros_process.start()

    main_loop(request_receiver)
 


if __name__ == "__main__":
    main()