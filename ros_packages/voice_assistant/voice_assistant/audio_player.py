import sys
import time
import wave
from queue import Queue
from threading import Lock, Event, Thread
from typing import Iterable

import pyaudio
import rclpy
from datatypes.srv import PlayAudioFromFile, PlayAudioFromSpeech, ClearPlaybackQueue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from public_api_client import public_voice_client
from . import util


class AudioEncoding:

    def __init__(
        self, bytes_per_sample: int, num_channels: int, frames_per_second: int
    ):
        self.bytes_per_sample = bytes_per_sample
        self.num_channels = num_channels
        self.frames_per_second = frames_per_second


class PlaybackItem:

    clear_threshold: int = 0
    clear_threshold_lock = Lock()

    def __init__(
        self,
        data: Iterable[bytes],
        encoding: AudioEncoding,
        pause_seconds: float,
        order: int,
    ):
        self.data = data  # the audio-data that is supposed to be played (as an iterable of bytes)
        self.encoding = encoding  # encoding of 'data'
        self.pause_seconds = pause_seconds  # seconds that should be waited after playback, to avoid cut-off
        self.order = (
            order  # indicates the number of elements aready added to the playback_queue
        )
        self.finished_playing = (
            Event()
        )  # setting this event will finished_playing playback, and the action-goal will abort

    def is_cleared(self) -> bool:
        with PlaybackItem.clear_threshold_lock:
            return self.order < PlaybackItem.clear_threshold

    def play(self) -> None:

        if self.is_cleared():
            self.finished_playing.set()
            return

        try:
            with util.surpress_stderr():
                pya = pyaudio.PyAudio()

                stream: pyaudio.Stream = pya.open(
                    format=pya.get_format_from_width(self.encoding.bytes_per_sample),
                    channels=self.encoding.num_channels,
                    rate=self.encoding.frames_per_second,
                    output=True,
                )

                for chunk in self.data:
                    if self.is_cleared():
                        break
                    stream.write(chunk)

                self.finished_playing.set()

                time.sleep(self.pause_seconds)

                stream.stop_stream()
                stream.close()
                pya.terminate()
        except OSError as e:
            # ToDo - Get better logging for non-ROS packages
            print(f"failed to playback audio: {e}", file=sys.stderr)
            pya.terminate()
            return


SPEECH_ENCODING = AudioEncoding(2, 1, 16000)
CHUNKS_PER_SECOND = 10
FRAMES_PER_CHUNK = SPEECH_ENCODING.frames_per_second // CHUNKS_PER_SECOND
BYTES_PER_FRAME = SPEECH_ENCODING.bytes_per_sample * SPEECH_ENCODING.num_channels
BYTES_PER_CHUNK = BYTES_PER_FRAME * FRAMES_PER_CHUNK


class AudioPlayerNode(Node):

    def __init__(self, playback_queue: Queue[PlaybackItem]):

        super().__init__("audio_player")

        self.playback_queue = playback_queue
        self.counter = 0
        self.counter_lock = Lock()

        play_audio_callback_group = MutuallyExclusiveCallbackGroup()
        clear_callback_group = MutuallyExclusiveCallbackGroup()

        # service for playing back speech (provided in the goal in form of text)
        self.play_audio_from_speech_service = self.create_service(
            PlayAudioFromSpeech,
            "play_audio_from_speech",
            self.play_audio_from_speech,
            callback_group=play_audio_callback_group,
        )

        # clears the playback queue
        self.clear_playback_queue_servie = self.create_service(
            ClearPlaybackQueue,
            "clear_playback_queue",
            self.clear_playback_queue,
            callback_group=clear_callback_group,
        )

        # service for playing back audio files (must be wav-format)
        self.play_audio_from_file_service = self.create_service(
            PlayAudioFromFile,
            "play_audio_from_file",
            self.play_audio_from_file,
            callback_group=play_audio_callback_group,
        )

        self.get_logger().info("Now running AUDIO PLAYER")

    def counter_next(self) -> int:
        with self.counter_lock:
            value = self.counter
            self.counter += 1
            return value


    
    def adjust_data_granularity(self, data: Iterable[bytes], target_bytes_per_chunk: int) -> Iterable[bytes]:
        """returns a new iterable, whose data chunks have the specified target size (in bytes)"""
        data_buffer: bytes = b''
        for chunk in data:
            # add data-chunk to the buffer
            data_buffer = data_buffer + chunk
            # iterate over the data stored in the data_buffer and
            # yield chunks of the specified target-size
            num_iters = len(data_buffer) // target_bytes_per_chunk
            for i in range(num_iters):
                offset = i*target_bytes_per_chunk
                yield data_buffer[offset:(offset + target_bytes_per_chunk)]
            # remove all data from the buffer, that was yielded during the loop above
            # (in case the length of the data in the buffer is not a multiple of the target-size
            # the remainder is kept and yielded during the next iteration)
            data_buffer = data_buffer[(num_iters*target_bytes_per_chunk):]
        # yield the remaining data
        if len(data_buffer) > 0:
            yield data_buffer


    
    def play_audio_from_file(
        self, request: PlayAudioFromFile.Request, response: PlayAudioFromFile.Response
    ) -> PlayAudioFromFile.Response:

        order = self.counter_next()

        with wave.open(request.filepath, "rb") as wf:

            encoding = AudioEncoding(
                wf.getsampwidth(), wf.getnchannels(), wf.getframerate()
            )

            frames_per_chunk = encoding.frames_per_second // CHUNKS_PER_SECOND

            data = []
            while True:
                chunk = wf.readframes(frames_per_chunk)
                data.append(chunk)
                if len(chunk) < frames_per_chunk:
                    break

        playback_item = PlaybackItem(data, encoding, 0.0, order)
        self.playback_queue.put(playback_item, True)

        if request.join:
            playback_item.finished_playing.wait()
        return response

    def play_audio_from_speech(
        self,
        request: PlayAudioFromSpeech.Request,
        response: PlayAudioFromSpeech.Response,
    ) -> PlayAudioFromSpeech.Response:

        order = self.counter_next()

        try:
            data = public_voice_client.text_to_speech(
                request.speech, request.gender, request.language
            )
        except Exception as e:
            self.get_logger().error(f"text_to_speech failed: {e}")
            return response
        data = self.adjust_data_granularity(data, BYTES_PER_CHUNK)

        playback_item = PlaybackItem(data, SPEECH_ENCODING, 0.2, order)
        self.playback_queue.put(playback_item, True)

        if request.join:
            playback_item.finished_playing.wait()
        return response

    def clear_playback_queue(
        self, _: PlayAudioFromFile.Request, response: PlayAudioFromFile.Response
    ) -> PlayAudioFromFile.Response:

        with PlaybackItem.clear_threshold_lock:
            with self.counter_lock:
                PlaybackItem.clear_threshold = self.counter

        return response


def main(args=None):

    playback_queue: Queue[PlaybackItem] = Queue()

    def audio_loop(playback_queue: Queue[PlaybackItem]) -> None:
        while True:
            playback_queue.get(True).play()

    Thread(target=audio_loop, args=(playback_queue,), daemon=True).start()

    rclpy.init()
    node = AudioPlayerNode(playback_queue)
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
