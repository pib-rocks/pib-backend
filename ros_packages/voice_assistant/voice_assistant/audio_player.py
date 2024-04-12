from collections import deque
import time
from typing import Any, Callable, Iterable
import wave
import pyaudio
from pyaudio import Stream
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.action.client import GoalStatus

from datatypes.action import PlayAudioFromFile, PlayAudioFromSpeech
from datatypes.srv import ClearPlaybackQueue

from threading import Lock, Event

from public_api_client import public_voice_client

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle



class AudioEncoding():

    def __init__(self, bytes_per_sample: int, num_channels: int, frames_per_second: int):
        self.bytes_per_sample = bytes_per_sample
        self.num_channels = num_channels
        self.frames_per_second = frames_per_second

class PlaybackItem():

    def __init__(self, goal_handle: ServerGoalHandle, create_result: Callable[[], Any], data: Iterable[bytes], encoding: AudioEncoding, pause_seconds: float):
        self.goal_handle = goal_handle # executing this goal_handle will start playback
        self.create_result = create_result, # creates the action-result, that that is returned for the goal
        self.data = data # the audio-data that is supposed to be played (as an iterable of bytes)
        self.encoding = encoding # encoding of 'data'
        self.pause_seconds = pause_seconds # seconds that should be waited after playback, to avoid cut-off
        self.interrupt = Event() # setting this event will interrupt playback, and the action-goal will abort



SPEECH_ENCODING = AudioEncoding(2, 1, 16000)
CHUNKS_PER_SECOND = 10



class AudioPlayerNode(Node):

    def __init__(self):

        super().__init__('audio_player')

        # requests to play audio are stored in here
        self.playback_queue: deque[PlaybackItem] = deque()
        self.playback_lock: Lock = Lock()

        # server for playing back audio files (must be wav-format)
        # the action creates no feedback and cannot be cancelled
        self.push_file_to_playback_queue_server = ActionServer(
            self, 
            PlayAudioFromFile, 
            'play_audio_from_file', 
            execute_callback = self.play_audio,
            handle_accepted_callback=self.handle_accepted_file_goal,
            cancel_callback = (lambda _ : CancelResponse.REJECT))

        # server for playing back speech (provided in the goal in form of text)
        # the action creates no feedback and cannot be cancelled
        self.push_speech_to_playback_queue_server = ActionServer(
            self, 
            PlayAudioFromSpeech, 
            'play_audio_from_speech', 
            execute_callback = self.play_audio,
            handle_accepted_callback=self.handle_accepted_speech_goal,
            cancel_callback = (lambda _ : CancelResponse.REJECT))
        
        # clears the playback queue
        self.clear_playback_queue_service = self.create_service(
            ClearPlaybackQueue, 
            'clear_playback_queue',
            self.clear_playback_queue)

        self.get_logger().info('Now running AUDIO PLAYER')



    def push_playback_item(self, playback_item: PlaybackItem) -> None:
        """adds an item to the playback queue and start execution of its goal, if no other goal is in queue"""
        with self.playback_lock:
            if not self.playback_queue: playback_item.goal_handle.execute()
            self.playback_queue.appendleft(playback_item)

    def pop_playback_item(self) -> None:
        """pops the current playback-item and start execution of the next one"""
        with self.playback_lock:
            self.playback_queue.pop()
            if self.playback_queue: self.playback_queue[-1].goal_handle.execute()

    def set_all_playback_items_inactive(self) -> None:
        """inactivate all queued items, thus stopping playback of current and all queued items"""
        with self.playback_lock:
            for item in self.playback_queue: item.interrupt.set()
                


    def handle_accepted_file_goal(self, goal_handle: ServerGoalHandle) -> None:

        request: PlayAudioFromFile.Goal = goal_handle.request
        filepath = request.filepath

        with wave.open(filepath, 'rb') as wf:

            encoding = AudioEncoding(
                wf.getsampwidth(),
                wf.getnchannels(),
                wf.getframerate())
            
            frames_per_chunk = encoding.frames_per_second // CHUNKS_PER_SECOND
            
            data = []
            while True:
                chunk = wf.readframes(frames_per_chunk)
                data.append(chunk)
                if len(chunk) < frames_per_chunk: break

        playback_item = PlaybackItem(goal_handle, lambda: PlayAudioFromFile.Result(), data, encoding, 0.0)
        self.push_playback_item(playback_item)



    def handle_accepted_speech_goal(self, goal_handle: ServerGoalHandle) -> None:

        request: PlayAudioFromSpeech.Goal = goal_handle.request
        data = public_voice_client.text_to_speech(request.speech, request.gender, request.language)

        playback_item = PlaybackItem(goal_handle, lambda: PlayAudioFromSpeech.Result(), data, SPEECH_ENCODING, 0.2)
        self.push_playback_item(playback_item)



    def play_audio(self, goal_handle: ServerGoalHandle):

        with self.playback_lock: playback_item = self.playback_queue[-1]
        encoding = playback_item.encoding
        
        pya = pyaudio.PyAudio()

        stream: Stream = pya.open(
            format=pya.get_format_from_width(encoding.bytes_per_sample), 
            channels=encoding.num_channels, 
            rate=encoding.frames_per_second, 
            output=True)
        
        for chunk in playback_item.data: 
            if playback_item.interrupt.is_set():
                goal_handle.abort()
                break
            stream.write(chunk)

        time.sleep(playback_item.pause_seconds)

        stream.stop_stream()
        stream.close()
        pya.terminate()

        self.pop_playback_item()

        if goal_handle.status != GoalStatus.STATUS_ABORTED: goal_handle.succeed()
        return playback_item.create_result[0]()



    def clear_playback_queue(self, _: ClearPlaybackQueue.Request, response: ClearPlaybackQueue.Response) -> ClearPlaybackQueue.Response:
        self.set_all_playback_items_inactive()
        return response



def main(args=None):

    rclpy.init()
    node = AudioPlayerNode()
    executor = MultiThreadedExecutor(8)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
