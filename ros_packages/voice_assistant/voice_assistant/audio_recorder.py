import os
import wave
from collections import deque
from threading import Lock, Event
from typing import Optional

import numpy as np
import pyaudio
import rclpy
from datatypes.action import RecordAudio
from datatypes.srv import GetMicConfiguration
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray

from public_api_client import public_voice_client
from . import util

# these values define the PCM encoding in which the recorded
# audio will be received
BYTES_PER_SAMPLE = 2

# TODO: this value should not be hardcoded, as the optimal value
# depends on the level of background noise.
SILENCE_VOLUME_THRESHOLD = 500

VOICE_ASSISTANT_DIRECTORY = os.getenv(
    "VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant"
)
OUTPUT_FILE_PATH = VOICE_ASSISTANT_DIRECTORY + "/audiofiles/output.wav"


class AudioRecorderNode(Node):

    def __init__(self):
        super().__init__("audio_recorder")

        # Set defaults for mic configuration
        self.mic_channels = None
        self.chunk_size = None
        self.audio_format = None
        self.sample_rate = None

        # Prepare service client
        self.ros_mic_configuration_client = self.create_client(
            GetMicConfiguration, "get_mic_configuration"
        )

        # Request & handle response
        if not self.request_mic_configuration():
            rclpy.shutdown()
            return

        self.token: Optional[str] = None

        self.goal_queue: deque[ServerGoalHandle] = deque()
        self.goal_queue_lock = Lock()

        # server for recording audio and transcribing it to text via the public‐API
        self.record_audio_server = ActionServer(
            self,
            RecordAudio,
            "record_audio",
            execute_callback=self.record_audio,
            handle_accepted_callback=self.handle_accepted_goal,
            cancel_callback=(lambda _: CancelResponse.ACCEPT),
        )

        self.get_token_subscription = self.create_subscription(
            String, "public_api_token", self.get_public_api_token_listener, 10
        )

        # buffer for incoming audio stream frames
        self.audio_chunks = deque()
        self.audio_chunks_lock = Lock()
        self.audio_chunk_event = Event()

        # subscribe to ROS2 audio_stream topic
        self.ros_audio_stream_subscription = self.create_subscription(
            Int16MultiArray, "audio_stream", self.audio_stream_callback, 10
        )

        self.get_logger().info("Now running AUDIO RECORDER")

    def request_mic_configuration(self, timeout_sec: float = 5.0) -> bool:
        """
        Call the get_mic_configuration service,
        populate self.mic_channels, self.chunk_size, self.audio_format and self.sample_rate,
        and return True on success (False on failure / timeout).
        """
        self.get_logger().info("Requesting mic configuration from server…")

        # Make sure that service already appeared
        if not self.ros_mic_configuration_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            self.get_logger().error(
                f"get_mic_configuration service not available after {timeout_sec}s"
            )
            return False

        # Create the request locally
        ros_mic_configuration_request = GetMicConfiguration.Request()

        # Send the request
        future = self.ros_mic_configuration_client.call_async(
            ros_mic_configuration_request
        )

        # Block and spin until we get a response (or timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        # Check for timeout / failure
        if not future.done():
            self.get_logger().error("Service call timed out")
            return False

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False

        # Store the received parameters
        self.mic_channels = resp.mic_channels
        self.chunk_size = resp.chunk_size
        self.audio_format = resp.audio_format
        self.sample_rate = resp.sample_rate

        self.get_logger().info(
            f"Got mic configuration:"
            f"channels = {self.mic_channels}, "
            f"chunk = {self.chunk_size}, "
            f"format = {self.audio_format}, "
            f"rate = {self.sample_rate}"
        )
        return True

    def get_public_api_token_listener(self, msg):
        """Listener for incoming public‐API token messages."""
        token = msg.data
        self.token = token

    def handle_accepted_goal(self, goal_handle: ServerGoalHandle) -> GoalResponse:
        """
        Place a goal into the queue and start execution if the queue was empty before.
        """
        with self.goal_queue_lock:
            if not self.goal_queue:
                goal_handle.execute()
            self.goal_queue.appendleft(goal_handle)
        return GoalResponse.ACCEPT

    def audio_stream_callback(self, msg: Int16MultiArray):
        """
        Callback for incoming Int16MultiArray audio data.
        Convert to bytes and push into the chunks deque.
        """
        chunk_bytes = np.array(msg.data, dtype=np.int16).tobytes()
        with self.audio_chunks_lock:
            self.audio_chunks.append(chunk_bytes)
            self.audio_chunk_event.set()

    def get_next_chunk(self) -> bytes:
        """
        Block until a chunk is available, then return it.
        """
        while True:
            self.audio_chunk_event.wait()
            with self.audio_chunks_lock:
                if self.audio_chunks:
                    chunk = self.audio_chunks.popleft()
                    if not self.audio_chunks:
                        self.audio_chunk_event.clear()
                    return chunk

    def is_silent(self, data_chunk: bytes) -> bool:
        """
        Check whether a chunk of frames is below the minimum volume threshold.
        """
        as_ints = np.frombuffer(data_chunk, dtype=np.int16)
        return np.abs(as_ints).mean() < SILENCE_VOLUME_THRESHOLD

    def create_result(self, text: str) -> RecordAudio.Result:
        """
        Create an action result and start execution of the next queued goal (if any).
        """
        result = RecordAudio.Result()
        result.transcribed_text = text

        # Remove the current goal from the queue and start the next one (if present)
        with self.goal_queue_lock:
            self.goal_queue.pop()
            if self.goal_queue:
                self.goal_queue[-1].execute()

        return result

    def record_audio(self, goal_handle: ServerGoalHandle) -> None:
        """
        Record audio until we see `max_silent_seconds_before` of silence before speech
        or `max_silent_seconds_after` of silence after speech has started.
        We compute silence in actual time by looking at chunk‐duration using the known
        sample rate (self.sample_rate), rather than relying on a fixed chunk‐count.
        Exactly one feedback object is returned (empty), indicating that recording has
        stopped and transcription has begun. The final result contains the transcribed text.
        """
        request: RecordAudio.Goal = goal_handle.request

        # Read thresholds (in seconds) from the goal
        max_silent_before_secs = request.max_silent_seconds_before
        max_silent_after_secs = request.max_silent_seconds_after

        # Prepare to accumulate silence time
        silent_time_accum = 0.0
        speech_started = False
        max_silent_target = max_silent_before_secs  # initially, measure "before" speech

        # Clear any buffered audio from previous runs
        with self.audio_chunks_lock:
            self.audio_chunks.clear()
            self.audio_chunk_event.clear()

        # Collect incoming chunks into this list
        chunks = []

        # Number of bytes per single frame (sample × channels)
        bytes_per_frame = BYTES_PER_SAMPLE * self.mic_channels

        # Loop until we accumulate enough silence to stop
        while True:
            if goal_handle.is_cancel_requested:
                # If the client canceled the goal, stop immediately
                goal_handle.canceled()
                return self.create_result("")

            # Block until we get the next chunk
            data_chunk: bytes = self.get_next_chunk()
            chunks.append(data_chunk)

            # Compute how many frames are in this chunk
            num_frames_in_chunk = len(data_chunk) // bytes_per_frame
            # Convert to duration in seconds
            chunk_duration_secs = float(num_frames_in_chunk) / float(self.sample_rate)

            # Determine if this chunk is "silent"
            if self.is_silent(data_chunk):
                # Accumulate silence time
                silent_time_accum += chunk_duration_secs

                # If we've reached (or exceeded) the target silence time, break
                if silent_time_accum >= max_silent_target:
                    break
            else:
                # Non‐silence => speech has started (or is ongoing)
                if not speech_started:
                    # First non‐silent chunk: switch to "after" threshold
                    speech_started = True
                    silent_time_accum = 0.0
                    max_silent_target = max_silent_after_secs
                else:
                    # Already in speech‐ongoing mode, so reset the "after‐speech" silence counter
                    silent_time_accum = 0.0

                # Continue collecting until we accumulate enough silence after speech
                continue

        # Recording phase has ended; notify client that we are transcribing
        goal_handle.publish_feedback(RecordAudio.Feedback())

        # Write the collected raw PCM chunks into a WAV file using the correct sample rate
        data_bytes = b"".join(chunks)
        try:
            with wave.open(OUTPUT_FILE_PATH, "wb") as wave_file:
                wave_file.setnchannels(self.mic_channels)
                wave_file.setsampwidth(BYTES_PER_SAMPLE)
                wave_file.setframerate(self.sample_rate)
                wave_file.writeframes(data_bytes)
        except Exception as e:
            self.get_logger().error(f"Could not write WAV: {e}")
            goal_handle.abort()
            return self.create_result("")

        # Read the WAV file back into memory for the STT API
        try:
            with open(OUTPUT_FILE_PATH, "rb") as f:
                wav_data = f.read()
        except Exception as e:
            self.get_logger().error(f"Could not read WAV for upload: {e}")
            goal_handle.abort()
            return self.create_result("")

        # Call the public‐API to transcribe, then finish the goal
        try:
            text = public_voice_client.speech_to_text(wav_data, self.token)
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
