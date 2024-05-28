from collections import deque
import os
import wave
from collections import deque
from threading import Lock

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from datatypes.action import RecordAudio

from threading import Lock

from public_api_client import public_voice_client
import wave

from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from public_api_client import public_voice_client
from . import util

# these values define the pcm-encoding, in which the recorded
# audio will be received
BYTES_PER_SAMPLE = 2
FRAMES_PER_SECOND = 16000
NUM_CHANNELS = 1
CHUNKS_PER_SECOND = 10

FRAMES_PER_CHUNK = FRAMES_PER_SECOND // CHUNKS_PER_SECOND

# TODO: his value should not be hardcoded, as the optimal value
# depends on the level of background noise.
SILENCE_VOLUME_THRESHOLD = 500

VOICE_ASSISTANT_DIRECTORY = os.getenv(
    "VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant"
)
OUTPUT_FILE_PATH = VOICE_ASSISTANT_DIRECTORY + "/audiofiles/output.wav"


class AudioRecorderNode(Node):

    def __init__(self):

        super().__init__("audio_recorder")

        self.goal_queue: deque[ServerGoalHandle] = deque()
        self.goal_queue_lock = Lock()

        # server for recording audio and transcribing it to text, via the public-api
        self.record_audio_server = ActionServer(
            self,
            RecordAudio,
            "record_audio",
            execute_callback=self.record_audio,
            handle_accepted_callback=self.handle_accepted_goal,
            cancel_callback=(lambda _: CancelResponse.ACCEPT),
        )

        self.get_logger().info("Now running AUDIO RECORDER")

    def handle_accepted_goal(self, goal_handle: ServerGoalHandle) -> GoalResponse:
        """place a goal into the queue and start execution if the queue was empty before"""
        with self.goal_queue_lock:
            if not self.goal_queue:
                goal_handle.execute()
            self.goal_queue.appendleft(goal_handle)

    def is_silent(self, data_chunk) -> bool:
        """Check whether a chunk of frmaes is below the minimum volume threshold"""
        as_ints = np.frombuffer(data_chunk, dtype=np.int16)
        return np.abs(as_ints).mean() < SILENCE_VOLUME_THRESHOLD

    def create_result(self, text: str) -> RecordAudio.Result:
        """create an action-result and initilaize execution of the next queued goal"""

        # create an result object and return
        result = RecordAudio.Result()
        result.transcribed_text = text

        # remove the current goal from the queue, and start execution of next queued goal
        # (if one is present)
        with self.goal_queue_lock:
            self.goal_queue.pop()
            if self.goal_queue:
                self.goal_queue[-1].execute()

        return result

    def record_audio(self, goal_handle: ServerGoalHandle) -> None:
        """
        record audio and transcribe it to text. The result of the goal will be the transcribed text.
        exactly one feedback object is returned. The object is empty and indicated, that recording
        was stopped and transcription has begun
        """

        request: RecordAudio.Goal = goal_handle.request

        # these values indicate, after how many silent chunks, the recording is interrupted,
        # the 'before' value is used, if speech has not started yet
        # the 'after' value if used, after speech has already started
        # generally, the 'before' value should be greater than the 'afer' value
        # speech is considered to have started, once the first non-silent chunk is recognized
        max_silent_chunks_before = request.max_silent_seconds_before * CHUNKS_PER_SECOND
        max_silent_chunks_after = request.max_silent_seconds_after * CHUNKS_PER_SECOND

        # the collected chunks of frames
        chunks = []
        # the current number of silent chunks in a row
        silent_chunks = 0
        # the maximum number of silent chunks allowed in a row
        max_silent_chunks = max_silent_chunks_before

        # create an pyaudio-input-stream for recording audio
        with util.surpress_stderr():
            pya = pyaudio.PyAudio()
            try:
                stream = pya.open(
                    format=pya.get_format_from_width(BYTES_PER_SAMPLE),
                    channels=NUM_CHANNELS,
                    rate=FRAMES_PER_SECOND,
                    input=True,
                    frames_per_buffer=FRAMES_PER_CHUNK,
                )

                # record audio, until too many silent chunks were detected in a row
                # or if cancellation of the goal was requested
                while silent_chunks < max_silent_chunks:
                    chunk = stream.read(FRAMES_PER_CHUNK, exception_on_overflow=False)
                    chunks.append(chunk)
                    if goal_handle.is_cancel_requested:
                        break
                    if self.is_silent(chunk):
                        silent_chunks += 1
                    else:
                        max_silent_chunks = max_silent_chunks_after
                        silent_chunks = 0

                # stop recording
                stream.stop_stream()
                stream.close()
                pya.terminate()
            except OSError as e:
                self.get_logger().error(f"failed to record audio: {e}")
                pya.terminate()
                goal_handle.abort()
                return self.create_result("")

        # if cancel is requested, stop execution here
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self.create_result("")

        # if not canceled, transcribe recorded audio to text (via public-api)
        # and return the transcription
        # from now on, cancel requests are ignored (maybe change this in future?)

        # notifiy client, that the recording-phase has ended
        goal_handle.publish_feedback(RecordAudio.Feedback())

        # collect binary data
        data = b"".join(chunks)

        # TODO: public-api should accept raw pcm data in future -> this step will be unnecessary
        with wave.open(OUTPUT_FILE_PATH, "wb") as wave_file:
            wave_file.setnchannels(NUM_CHANNELS)
            wave_file.setsampwidth(BYTES_PER_SAMPLE)
            wave_file.setframerate(FRAMES_PER_SECOND)
            wave_file.writeframes(data)
            wave_file.close()
        with open(OUTPUT_FILE_PATH, "rb") as f:
            data = f.read()

        # transcribe the audio data
        try:
            text = public_voice_client.speech_to_text(data)
        except Exception as e:
            self.get_logger().error(f"failed speech_to_text: {e}")
            goal_handle.abort()
            return self.create_result("")

        goal_handle.succeed()
        return self.create_result(text)


def main(args=None):
    rclpy.init()
    node = AudioRecorderNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
