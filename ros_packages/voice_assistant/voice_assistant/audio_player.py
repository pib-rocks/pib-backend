"""Playback for spoken clauses.

``first_audio`` is the first non-empty PCM chunk handed to the audio device
for the current playback burst. ``last_audio`` is stamped once that burst's
queue is empty after the last queued item has been written (including an
empty join barrier, which is not itself audio). The burst flag is then
re-armed so the next turn can record its own first chunk.

The completion event is set only after ``last_audio`` has been handed to the
trace publisher, so the voice-assistant node can observe it before ``turn_end``.
Neither stamp adds a sleep or changes what is written.
"""

import io
import logging
import os
import sys
import time
import wave
from queue import Empty, Queue
from threading import Lock, Event, Thread
from typing import Iterable, Iterator, Optional

try:
    import pyaudio
except ImportError:
    pyaudio = None
import requests
import rclpy
from datatypes.srv import PlayAudioFromFile, PlayAudioFromSpeech, ClearPlaybackQueue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from pib_hermes_config.turn_taking import clauses_to_synthesize
from pib_hermes_config.visible_state import (
    VOICE_USING_FALLBACK_TOPIC,
    engine_is_fallback,
)
from pib_hermes_config.voice_backends import CLOUD_TTS_ROUTE, resolve_tts_route
from public_api_client import public_voice_client
from voice_assistant.cloud_token import CloudTokenError, resolve_cloud_token
from voice_assistant.pib_cloud_stream import pib_cloud_base_url
from voice_assistant.turn_latency import TURN_TRACE_TOPIC, format_event
from . import util
from .tts_synthesis import SupertoneTTSEngine

logger = logging.getLogger(__name__)

# Join waits must not block forever; a timeout is not proof the device stopped.
JOIN_WAIT_TIMEOUT_SECONDS = 120.0

#: pib.Cloud speech endpoint. The base URL is the one the chat client uses,
#: from ``PIB_CLOUD_BASE_URL``. The response is a WAV container, 16 kHz PCM16
#: little-endian, mono, so the existing wave clause fragmenting applies.
CLOUD_SPEECH_PATH = "/audio/speech"
CLOUD_TTS_TIMEOUT_SECONDS = 30.0
#: Statuses that mean the credential or the budget is done for this turn.
_CLOUD_TTS_AUTH_STATUS_CODES = frozenset({401, 403})
_CLOUD_TTS_BUDGET_STATUS_CODE = 402


class CloudTTSError(Exception):
    """pib.Cloud speech did not produce audio. ``reason`` is the fallback cause."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def cloud_speech_url(endpoint_base: Optional[str] = None) -> str:
    """Full URL for ``POST /v1/audio/speech``."""
    base = pib_cloud_base_url(endpoint_base)
    if base.endswith(CLOUD_SPEECH_PATH):
        return base
    if not base.endswith("/v1"):
        base = base + "/v1"
    return base + CLOUD_SPEECH_PATH


def cloud_language_code(value: object) -> str:
    """``de`` or ``en``. The endpoint accepts no third language."""
    text = value.strip().lower() if isinstance(value, str) else ""
    if text.startswith("en") or "engl" in text:
        return "en"
    return "de"


def cloud_voice(value: object) -> str:
    """``F1`` or ``M1``, from the personality's gender or a stored voice id."""
    text = value.strip().lower() if isinstance(value, str) else ""
    return "M1" if text.startswith("m") else "F1"


# The audio thread is the only writer. The service thread only enqueues items.
_trace_emitter = None
_playback_queue: Optional[Queue] = None
_burst_started = False


def set_trace_emitter(fn) -> None:
    """Install the audio-player node's publisher. None restores the no-op."""
    global _trace_emitter
    _trace_emitter = fn


def bind_playback_queue(playback_queue: Queue) -> None:
    global _playback_queue
    _playback_queue = playback_queue


def _emit_stage(stage: str) -> None:
    emitter = _trace_emitter
    if emitter is None:
        return
    try:
        emitter(format_event(stage, source="audio_player"))
    except Exception:
        logger.warning("turn trace emit failed for stage %s", stage)


def _note_pcm_written() -> None:
    """First non-empty chunk of a burst. Later chunks in the burst do not repeat it."""
    global _burst_started
    if _burst_started:
        return
    _burst_started = True
    _emit_stage("first_audio")


def _emit_last_audio_if_drained() -> None:
    """Stamp last_audio once the queue is empty, then re-arm the burst."""
    global _burst_started
    if not _burst_started:
        return
    playback_queue = _playback_queue
    if playback_queue is None:
        return
    try:
        drained = playback_queue.empty()
    except Exception:
        return
    if not drained:
        return
    _burst_started = False
    _emit_stage("last_audio")


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

    @staticmethod
    def order_is_stale(order: int) -> bool:
        with PlaybackItem.clear_threshold_lock:
            return order < PlaybackItem.clear_threshold

    def play(self) -> None:

        if self.is_cleared() or pyaudio is None:
            _emit_last_audio_if_drained()
            self.finished_playing.set()
            return

        pya = None
        stream = None
        try:
            with util.surpress_stderr():
                pya = pyaudio.PyAudio()

                stream = pya.open(
                    format=pya.get_format_from_width(self.encoding.bytes_per_sample),
                    channels=self.encoding.num_channels,
                    rate=self.encoding.frames_per_second,
                    output=True,
                )

                for chunk in self.data:
                    if self.is_cleared():
                        break
                    stream.write(chunk)
                    if chunk:
                        _note_pcm_written()

                time.sleep(self.pause_seconds)
        except Exception as e:
            print(f"failed to playback audio: {e}", file=sys.stderr)
        finally:
            _emit_last_audio_if_drained()
            self.finished_playing.set()
            if stream is not None:
                try:
                    stream.stop_stream()
                    stream.close()
                except Exception:
                    pass
            if pya is not None:
                try:
                    pya.terminate()
                except Exception:
                    pass


SPEECH_ENCODING = AudioEncoding(2, 1, 44100)
CHUNKS_PER_SECOND = 10
FRAMES_PER_CHUNK = SPEECH_ENCODING.frames_per_second // CHUNKS_PER_SECOND
BYTES_PER_FRAME = SPEECH_ENCODING.bytes_per_sample * SPEECH_ENCODING.num_channels
BYTES_PER_CHUNK = BYTES_PER_FRAME * FRAMES_PER_CHUNK


class AudioPlayerNode(Node):

    def __init__(self, playback_queue: Queue[PlaybackItem]):

        super().__init__("audio_player")
        self.token: Optional[str] = None
        self.playback_queue = playback_queue
        bind_playback_queue(playback_queue)
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

        self.get_token_subscription = self.create_subscription(
            String, "public_api_token", self.get_public_api_token_listener, 10
        )

        self.tts_engine = SupertoneTTSEngine()
        self._published_fallback = None
        self.voice_using_fallback_publisher = self.create_publisher(
            Bool, VOICE_USING_FALLBACK_TOPIC, 10
        )
        self._install_turn_trace_emitter()

        self.get_logger().info("Now running AUDIO PLAYER")

    def _install_turn_trace_emitter(self) -> None:
        """Publish first_audio / last_audio. A failure must not stop playback."""
        try:
            publisher = self.create_publisher(String, TURN_TRACE_TOPIC, 10)
            self._turn_trace_publisher = publisher

            def emit(raw: str) -> None:
                message = String()
                message.data = raw
                publisher.publish(message)

            set_trace_emitter(emit)
        except Exception as exc:
            self.get_logger().warning(f"turn trace emitter was not installed: {exc}")

    def get_public_api_token_listener(self, msg):
        token = msg.data
        self.token = token

    def counter_next(self) -> int:
        with self.counter_lock:
            value = self.counter
            self.counter += 1
            return value

    def adjust_data_granularity(
        self, data: Iterable[bytes], target_bytes_per_chunk: int
    ) -> Iterable[bytes]:
        """returns a new iterable, whose data chunks have the specified target size (in bytes)"""
        data_buffer: bytes = b""
        for chunk in data:
            # add data-chunk to the buffer
            data_buffer = data_buffer + chunk
            # iterate over the data stored in the data_buffer and
            # yield chunks of the specified target-size
            num_iters = len(data_buffer) // target_bytes_per_chunk
            for i in range(num_iters):
                offset = i * target_bytes_per_chunk
                yield data_buffer[offset : (offset + target_bytes_per_chunk)]
            # remove all data from the buffer, that was yielded during the loop above
            # (in case the length of the data in the buffer is not a multiple of the target-size
            # the remainder is kept and yielded during the next iteration)
            data_buffer = data_buffer[(num_iters * target_bytes_per_chunk) :]
        # yield the remaining data
        if len(data_buffer) > 0:
            yield data_buffer

    def play_audio_from_file(
        self, request: PlayAudioFromFile.Request, response: PlayAudioFromFile.Response
    ) -> PlayAudioFromFile.Response:

        order = self.counter_next()

        if not os.path.exists(request.filepath):
            self.get_logger().warning(
                f"Audio file not found '{request.filepath}', skipping file playback."
            )
            return response

        try:
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

            if PlaybackItem.order_is_stale(order):
                self.get_logger().info(
                    "play_audio_from_file: admission fence dropped stale file playback (order=%s)"
                    % (order,)
                )
                return response

            playback_item = PlaybackItem(data, encoding, 0.0, order)
            if not self._enqueue_current(playback_item):
                return response

            if request.join:
                self._wait_playback_finished(
                    playback_item, context="play_audio_from_file"
                )
        except Exception as e:
            self.get_logger().error(
                f"Error reading audio file '{request.filepath}': {e}"
            )

        return response

    def play_audio_from_speech(
        self,
        request: PlayAudioFromSpeech.Request,
        response: PlayAudioFromSpeech.Response,
    ) -> PlayAudioFromSpeech.Response:

        order = self.counter_next()
        speech = request.speech or ""
        if not speech.strip():
            # A barrier, so the caller can wait until earlier clauses finish.
            if PlaybackItem.order_is_stale(order):
                self.get_logger().info(
                    "play_audio_from_speech: admission fence dropped empty join barrier (order=%s)"
                    % (order,)
                )
                return response
            playback_item = PlaybackItem([b""], SPEECH_ENCODING, 0.0, order)
            if not self._enqueue_current(playback_item):
                return response
            if request.join:
                self._wait_playback_finished(
                    playback_item, context="play_audio_from_speech_barrier"
                )
            return response

        route = resolve_tts_route(getattr(request, "tts_engine", "") or "")
        if route not in ("local", CLOUD_TTS_ROUTE):
            self.get_logger().error(
                f"tts backend {request.tts_engine} is not the local Supertone "
                "engine and has no speech client"
            )
            return response

        # A fallback is reported once, for the turn. The token is read once for
        # the same reason: the key store is not consulted per clause.
        self._cloud_tts_fallback_noted = False
        self._cloud_tts_turn_token = None

        pieces = clauses_to_synthesize(speech)
        last_item = None
        for index, piece in enumerate(pieces):
            # Every clause retains the request admission made before synthesis.
            if PlaybackItem.order_is_stale(order):
                self.get_logger().info(
                    "play_audio_from_speech: admission fence skipped clause (order=%s piece_index=%s)"
                    % (order, index)
                )
                return response
            # Queue each clause as soon as it is synthesized so playback of
            # the first clause overlaps synthesis of the next one.
            pause = 0.2 if index == len(pieces) - 1 else 0.0
            try:
                data = self._synthesize_clause(piece, request)
            except Exception as exc:
                self.get_logger().error(f"clause synthesis failed: {exc}")
                return response
            if PlaybackItem.order_is_stale(order):
                self.get_logger().info(
                    "play_audio_from_speech: admission fence dropped post-synthesis clause (order=%s piece_index=%s)"
                    % (order, index)
                )
                return response
            data = self._fence_pcm_iterable(data, order)
            playback_item = PlaybackItem(data, SPEECH_ENCODING, pause, order)
            if not self._enqueue_current(playback_item):
                return response
            last_item = playback_item

        if request.join and last_item is not None:
            self._wait_playback_finished(last_item, context="play_audio_from_speech")
        return response

    def _synthesize_clause(self, text: str, request: PlayAudioFromSpeech.Request):
        """One clause of audio, from pib.Cloud when that engine is chosen.

        A cloud failure never leaves the personality mute: the clause is
        spoken by the local Supertone engine instead, after one logged reason
        for the turn.
        """
        route = resolve_tts_route(getattr(request, "tts_engine", "") or "")
        if route == CLOUD_TTS_ROUTE:
            try:
                data = self._synthesize_cloud_clause(text, request)
                self._publish_engine_fallback("")
            except CloudTTSError as exc:
                self._note_cloud_tts_fallback(exc.reason)
                data = self._synthesize_local_clause(text, request)
        else:
            data = self._synthesize_local_clause(text, request)
        return self.adjust_data_granularity(data, BYTES_PER_CHUNK)

    def _synthesize_local_clause(self, text: str, request: PlayAudioFromSpeech.Request):
        try:
            wav_bytes = self.tts_engine.synthesize(
                text=text,
                language=request.language or "auto",
                voice=request.gender or "F1",
                emotion="expressive",
            )
            self._publish_engine_fallback(
                getattr(self.tts_engine, "active_backend", "")
            )
            return self._pcm_chunks(wav_bytes)
        except Exception as exc:
            self.get_logger().warning(
                "Local Supertone TTS synthesis error, attempting public voice "
                f"client: {exc}"
            )
            return public_voice_client.text_to_speech(
                text, request.gender, request.language, getattr(self, "token", None)
            )

    def _synthesize_cloud_clause(
        self, text: str, request: PlayAudioFromSpeech.Request
    ) -> list[bytes]:
        """WAV from ``POST /v1/audio/speech``, as PCM chunks ready to play.

        The request carries the clause text, ``de``/``en`` and ``F1``/``M1``,
        and the pib.Cloud bearer token from the key store: the same route the
        chat client reads. Any failure raises ``CloudTTSError`` with a stable
        reason, so the caller can fall back deterministically.
        """
        try:
            token = self._cloud_tts_token()
        except CloudTokenError:
            raise CloudTTSError("cloud_token_unavailable") from None
        body = {
            "input": text,
            "language": cloud_language_code(request.language),
            "voice": cloud_voice(request.gender),
        }
        headers = {
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "Accept": "audio/wav",
        }
        try:
            response = requests.post(
                cloud_speech_url(),
                headers=headers,
                json=body,
                timeout=CLOUD_TTS_TIMEOUT_SECONDS,
            )
        except Exception as exc:
            raise CloudTTSError("cloud_transport_error") from exc

        status = getattr(response, "status_code", None)
        if status in _CLOUD_TTS_AUTH_STATUS_CODES:
            raise CloudTTSError(f"cloud_auth_rejected:{status}")
        if status == _CLOUD_TTS_BUDGET_STATUS_CODE:
            raise CloudTTSError(f"cloud_budget_exhausted:{status}")
        if status != 200:
            raise CloudTTSError(f"cloud_http_error:{status}")

        wav_bytes = getattr(response, "content", None) or b""
        if not wav_bytes:
            raise CloudTTSError("cloud_empty_body")
        try:
            chunks = self._pcm_chunks(wav_bytes)
        except Exception as exc:
            raise CloudTTSError("cloud_invalid_wav") from exc
        if not chunks:
            raise CloudTTSError("cloud_empty_audio")
        return chunks

    def _cloud_tts_token(self) -> str:
        """The key-store token, read once per turn and reused for its clauses."""
        cached = getattr(self, "_cloud_tts_turn_token", None)
        if cached:
            return cached
        token = resolve_cloud_token()
        self._cloud_tts_turn_token = token
        return token

    def _note_cloud_tts_fallback(self, reason: str) -> None:
        """Log why this turn left pib.Cloud speech, once per turn."""
        if getattr(self, "_cloud_tts_fallback_noted", False):
            return
        self._cloud_tts_fallback_noted = True
        self.get_logger().warning(
            f"pib.Cloud speech backend failed ({reason}); falling back to the "
            "local Supertone engine for this turn"
        )

    @staticmethod
    def _pcm_chunks(wav_bytes: bytes) -> list[bytes]:
        """PCM16 frames of a WAV container, split into player-sized chunks."""
        buf = io.BytesIO(wav_bytes)
        with wave.open(buf, "rb") as wf:
            raw_pcm = wf.readframes(wf.getnframes())
        return [
            raw_pcm[i : i + BYTES_PER_CHUNK]
            for i in range(0, len(raw_pcm), BYTES_PER_CHUNK)
        ]

    def _fence_pcm_iterable(self, data: Iterable[bytes], order: int) -> Iterator[bytes]:
        """Stop yielding PCM if this admission generation was cancelled."""
        for chunk in data:
            if PlaybackItem.order_is_stale(order):
                return
            yield chunk

    def _enqueue_current(self, item: PlaybackItem) -> bool:
        # Same lock order as clear: counter -> threshold -> queue. No engine or
        # device work is performed under this admission lock.
        with self.counter_lock:
            if item.is_cleared():
                item.finished_playing.set()
                return False
            self.playback_queue.put(item, True)
            return True

    def _wait_playback_finished(self, item: PlaybackItem, context: str) -> None:
        next_report = time.monotonic() + JOIN_WAIT_TIMEOUT_SECONDS
        while not item.finished_playing.wait(timeout=0.05):
            if item.is_cleared():
                return  # cancellation ACK, not completed device playback
            if time.monotonic() >= next_report:
                self.get_logger().warning(
                    f"playback join still pending ({context}); no drain ACK; "
                    "waiting for actual completion or explicit cancellation"
                )
                next_report = time.monotonic() + JOIN_WAIT_TIMEOUT_SECONDS

    def _publish_engine_fallback(self, active_backend: object) -> None:
        fallback = engine_is_fallback(active_backend)
        if fallback == self._published_fallback:
            return
        self._published_fallback = fallback
        message = Bool()
        message.data = fallback
        self.voice_using_fallback_publisher.publish(message)

    def clear_playback_queue(
        self, _: ClearPlaybackQueue.Request, response: ClearPlaybackQueue.Response
    ) -> ClearPlaybackQueue.Response:

        self.get_logger().info(
            "clear_playback_queue: accepting cancellation (ACK is not proof device stopped)"
        )
        with self.counter_lock:
            with PlaybackItem.clear_threshold_lock:
                PlaybackItem.clear_threshold = self.counter
                threshold = PlaybackItem.clear_threshold
            drained = 0
            while True:
                try:
                    pending = self.playback_queue.get_nowait()
                except Empty:
                    break
                pending.finished_playing.set()
                drained += 1

        self.get_logger().info(
            "clear_playback_queue: ACK threshold=%s drained=%s" % (threshold, drained)
        )
        return response


def main(args=None):

    playback_queue: Queue[PlaybackItem] = Queue()
    bind_playback_queue(playback_queue)

    def audio_loop(playback_queue: Queue[PlaybackItem]) -> None:
        while True:
            playback_queue.get(True).play()
            # play() already stamps last_audio when the queue is empty before
            # it signals completion. This second check covers a clear that
            # returned early, then re-arms the burst for the next turn.
            _emit_last_audio_if_drained()

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
