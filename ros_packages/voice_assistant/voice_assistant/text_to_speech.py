import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from datatypes.srv import TextToSpeechClear, TextToSpeechPush

from multiprocessing import Process, Pipe
from multiprocessing.connection import Connection
import time
from threading import Lock, Event

from pib_voice.voice import play_audio_from_text



class TextRequest:

    def __init__(self, text: str, voice: str, join: bool):
        self.text = text
        self.voice = voice
        self.join = join



class TextToSpeechNode(Node):

    def __init__(self, ros_to_main: Connection):

        super().__init__('text_to_speech')

        # connection for communicating with 'main'
        # used to send requests to stop the current worker
        # and get a connection to a new worker from main
        self.ros_to_main: Connection = ros_to_main

        # receive connection to inital worker
        ros_to_main.send(None)
        self.ros_to_worker: Connection = ros_to_main.recv()
        # for checking if the worker has changed (i.e. the queue was cleared)
        # while joining (i.e. waiting for audio to finish playing)
        self.worker_connection_stale: bool = False
        # only access 'ros_to_worker' and 'worker_connection_stale' while holding this lock 
        self.ros_to_worker_lock = Lock()

        self.get_logger().info('Now running TTS')

        # Service for pushing text to the queue
        self.push_text_service = self.create_service(
            TextToSpeechPush, 
            'tts_push',
            self.push_text,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        # Service for clearing the queue
        self.clear_service = self.create_service(
            TextToSpeechClear, 
            'tts_clear',
            self.clear,
            callback_group=MutuallyExclusiveCallbackGroup())
        
    def push_text(self, request: TextToSpeechPush.Request, response: TextToSpeechPush.Response) -> TextToSpeechPush.Response:
        
        # add a text-message to the queue
        with self.ros_to_worker_lock:
            self.ros_to_worker.send(TextRequest(request.text, request.voice, request.join))
            self.worker_connection_stale = False

        # return immediately (without waiting for audio to finish playing)
        # if joining not requested
        if not request.join: return response

        # wait for audio to finish playing
        while True:
            with self.ros_to_worker_lock:
                if self.worker_connection_stale: 
                    return response
                if self.ros_to_worker.poll():
                    self.ros_to_worker.recv()
                    return response
            time.sleep(0.1)

    def clear(self, request: TextToSpeechClear.Request, response: TextToSpeechClear.Response) -> TextToSpeechClear.Response:
        
        # terminate current worker and create new connection
        # -> current audio stops playing and queued text is cleared
        with self.ros_to_worker_lock:
            # send arbitrary value to main, in order to request termination of current worker
            self.ros_to_main.send(None)
            # indicate that the worker has changed for (potential) waiting callback
            self.worker_connection_stale = True
            # receive connection to freshly created worker
            self.ros_to_worker: Connection = self.ros_to_main.recv()

        return response
        


def ros_target(to_main: Connection) -> None:
    
    rclpy.init()
    node = TextToSpeechNode(to_main)
    # only a maximum of 2 threads may run in parallel, since
    # there are 2 mutex-callback-groups
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()



def main_target(main_to_ros: Connection) -> None:

    worker: Process = None

    while True:

        # receiving arbitrary value from ros = ros requests 
        # main to terminate current worker
        main_to_ros.recv()

        # worker is initially None -> must not be terminated
        if worker is not None: worker.terminate()
        # create fresh worker with a fresh pipe to ros
        worker_to_ros, ros_to_worker = Pipe()
        worker = Process(target=worker_target, args=(worker_to_ros,))
        worker.start()
        # send the connection to the fresh worker to ros
        main_to_ros.send(ros_to_worker)

        

def worker_target(worker_to_ros: Connection) -> None:

    while True:

        # receive next text from ros and play back the text as audio
        request: TextRequest = worker_to_ros.recv()
        play_audio_from_text(request.text, request.voice)
        # if joining is requested, notify ros, that audio has finished playing
        # only one callback can wait at a time (since push-callbacks are placed
        # in a mutex-callback-group, so it suffices to send an arbitrary value
        if request.join: worker_to_ros.send(None)



def main(args=None) -> None:

    ros_to_main, main_to_ros = Pipe()
    ros_process = Process(target=ros_target, args=(ros_to_main,))
    ros_process.start()

    main_target(main_to_ros)



if __name__ == "__main__":
    main()
