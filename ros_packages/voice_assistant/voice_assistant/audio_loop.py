# audio_loop.py
#
# Gemini live audio loop that:
# - Subscribes to a ROS audio topic and forwards PCM chunks to Gemini live session.
# - Receives audio (TTS) and transcript events back from Gemini.
# - Streams user/assistant transcripts into ChatNode via the CreateOrUpdateChatMessage srv
#   so the chat UI updates live while Gemini speaks.
# - Handles Stop/Start robustly by closing the live session and joining the worker thread.

import asyncio
import os
import logging
import threading
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16MultiArray
from pib_api_client import voice_assistant_client

import pyaudio
from google import genai

import wave
from pathlib import Path
from datetime import datetime
from asyncio import QueueEmpty

# service API to ChatNode (to avoid duplicating DB/publish logic here)
from datatypes.srv import CreateOrUpdateChatMessage

import queue        
import time         

# ——— Logging ———
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("GeminiAudioLoop")

# ——— Model + audio IO constants ———
pya = pyaudio.PyAudio()
FORMAT = pyaudio.paInt16
CHANNELS = 1       # send mono to Gemini
SEND_SAMPLE_RATE = 16000   # recorder publishes 16 kHz mono
RECEIVE_SAMPLE_RATE = 24000  # model replies at 24 kHz
CHUNK_SIZE = 1024

MODEL = "gemini-2.5-flash-native-audio-preview-09-2025"
CONFIG = {
    "response_modalities": ["AUDIO"],           # request synthesized speech back
    "input_audio_transcription": {},            # get user (input) transcript stream
    "output_audio_transcription": {},           # get assistant (output) transcript stream
}
ROS_AUDIO_TOPIC = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")


# ——————————————————————————————————————————
#         ROS subscriber bridge (thread)
# ——————————————————————————————————————————
class RosAudioBridge:
    """
    Subscribes to a ROS topic carrying PCM16 mono @16k (Int16MultiArray or AudioData)
    and forwards chunks into an asyncio.Queue that send_realtime() consumes.
    Runs in its own thread with a SingleThreadedExecutor.
    """

    def __init__(self, topic: str, loop: asyncio.AbstractEventLoop, out_queue: asyncio.Queue):
        self._topic = topic
        self._loop = loop
        self._out_queue = out_queue
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._started_evt = threading.Event()

    def start(self):
        """Start ROS subscriber thread and wait until it’s actually listening."""
        self._thread = threading.Thread(target=self._run, name="RosAudioBridge", daemon=True)
        self._thread.start()
        self._started_evt.wait(timeout=3.0)

    def stop(self):
        """Signal shutdown and join quickly (best-effort)."""
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _run(self):
        try:
            # Optional support for audio_common_msgs/AudioData (bytes)
            AudioData = None
            try:
                from audio_common_msgs.msg import AudioData as _AudioData  # type: ignore
                AudioData = _AudioData
            except Exception:
                pass

            # Ensure a ROS context exists in this thread
            if not rclpy.ok():
                rclpy.init()

            class _Node(Node):
                """Tiny ROS node that receives PCM and forwards to asyncio queue."""
                def __init__(self, topic, loop, queue, stop_evt):
                    super().__init__("ros_audio_bridge")
                    self._loop = loop
                    self._queue = queue
                    self._stop_evt = stop_evt
                    self._subs = []
                    self._subs.append(
                        self.create_subscription(Int16MultiArray, topic, self._cb_int16, 10)
                    )
                    if AudioData is not None:
                        self._subs.append(
                            self.create_subscription(AudioData, topic, self._cb_bytes, 10)
                        )
                    self.get_logger().info(f"Subscribed to '{topic}' for PCM16 mono @16k")

                def _enqueue(self, payload: bytes):
                    """Push a PCM payload into the asyncio queue (thread-safe)."""
                    if self._stop_evt.is_set():
                        return

                    async def _put():
                        await self._queue.put({"data": payload, "mime_type": "audio/pcm"})

                    try:
                        fut = asyncio.run_coroutine_threadsafe(_put(), self._loop)
                        fut.result(timeout=0.25)
                    except Exception:
                        # If the loop is busy, drop late chunks rather than blocking.
                        pass

                def _cb_int16(self, msg: Int16MultiArray):
                    try:
                        arr = np.asarray(msg.data, dtype=np.int16)
                        self._enqueue(arr.tobytes())
                    except Exception as e:
                        self.get_logger().error(f"Int16MultiArray convert error: {e}")

                def _cb_bytes(self, msg):  # AudioData
                    try:
                        self._enqueue(bytes(msg.data))
                    except Exception as e:
                        self.get_logger().error(f"AudioData forward error: {e}")

            node = _Node(self._topic, self._loop, self._out_queue, self._stop_evt)
            self._started_evt.set()

            executor = SingleThreadedExecutor(context=rclpy.get_default_context())
            executor.add_node(node)
            while not self._stop_evt.is_set():
                executor.spin_once(timeout_sec=0.1)
            executor.shutdown()
            node.destroy_node()

        except Exception:
            logger.exception("RosAudioBridge crashed")


# ——————————————————————————————————————————
#                Main audio loop
# ——————————————————————————————————————————
class GeminiAudioLoop:
    """
    Background worker that:
      - Maintains a Gemini live audio session.
      - Sends ROS PCM input upstream; receives PCM out + transcripts downstream.
      - For each transcript slice (user or assistant), calls ChatNode service to
        create/update the message so the UI reflects text while audio is speaking.
      - Supports Stop/Start: Stop sets an event, attempts to close the live session,
        cancels all tasks, and joins the thread so a new run can start cleanly.
    """

    def __init__(self, api_key: str = "") -> None:
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._is_listening = False
        self._chat_id: Optional[str] = None

        # Initialized in run()
        self.audio_in_queue: asyncio.Queue[bytes]
        self.out_queue: asyncio.Queue[bytes]
        self.session = None          # gemini live session (aio client)
        self.playback_stream = None  # PyAudio output stream
        self.api_key = api_key

        # ROS bridge handle
        self._ros_bridge: Optional[RosAudioBridge] = None

        # Logging / turns (optional: write input/output wavs)
        self._turn_id = 0
        self._in_wav: Optional[wave.Wave_write] = None
        self._out_wav: Optional[wave.Wave_write] = None
        self._log_lock: Optional[asyncio.Lock] = None
        self._log_input_dir = Path(os.getenv("AUDIO_LOG_INPUT_DIR", "input"))
        self._log_output_dir = Path(os.getenv("AUDIO_LOG_OUTPUT_DIR", "output"))

        # Service client state (for streaming ChatMessages to ChatNode)
        self._srv_node: Optional[Node] = None
        self._srv_client = None
        self._accum_text: str = ""
        self._last_pib_message_id: str = ""
        self._current_role: Optional[str] = None  # "user" | "assistant" | None

        # Chat update worker (so DB/UI updates don't block audio)
        self._chat_queue: "queue.Queue[CreateOrUpdateChatMessage.Request]" = queue.Queue(
            maxsize=32
        )
        self._chat_worker: Optional[threading.Thread] = None
        self._last_srv_call_time: float = 0.0
        self._srv_call_min_interval: float = 0.15 # seconds, 150 ms default

        # Event loop reference (used by stop() to close session)
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    @property
    def is_listening(self) -> bool:
        return self._is_listening

    # ---------- lifecycle ----------

    def start(self, chat_id) -> None:
        """
        Start the Gemini audio loop in a background thread.
        If already running, this is a no-op (defensive).
        """
        if self._is_listening:
            logger.info("GeminiAudioLoop is already running.")
            return

        self._chat_id = chat_id
        self._stop_event.clear()

        # Start chat worker first so it can consume requests
        self._start_chat_worker()

        self._thread = threading.Thread(target=self._run_thread, daemon=True)
        self._thread.start()
        self._is_listening = True
        logger.info("GeminiAudioLoop: started")

    def stop(self, join_timeout: float = 5.0) -> None:
        """
        Signal the loop to stop and join the thread.
        - Sets the stop flag.
        - Tries to close the live session (from this thread) via run_coroutine_threadsafe.
        - Joins the worker thread (best-effort) so Start can run cleanly after.
        """
        if not self._is_listening:
            logger.info("GeminiAudioLoop is not running.")
            return

        self._stop_event.set()
        # Wake chat worker so it can exit
        try:
            self._chat_queue.put_nowait(None)  # sentinel
        except Exception:
            pass

        if self._chat_worker:
            self._chat_worker.join(timeout=join_timeout)
            if self._chat_worker.is_alive():
                logger.warning("Chat worker thread did not stop cleanly.")
            else:
                self._chat_worker = None

        # Reset per-stream accumulators to avoid accidental reuse next run
        self._accum_text = ""
        self._last_pib_message_id = ""
        self._current_role = None

        # Try to close the live session from here (async -> thread-safe)
        if self._loop and self.session is not None:
            try:
                async def _close():
                    try:
                        await self.session.close()
                    except Exception:
                        pass
                asyncio.run_coroutine_threadsafe(_close(), self._loop)
            except Exception:
                pass

        if self._thread:
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                logger.warning("GeminiAudioLoop thread did not stop cleanly.")
            else:
                self._thread = None

        self._is_listening = False
        logger.info("GeminiAudioLoop: stopped")

    def _run_thread(self):
        """
        Thread entrypoint: create an isolated asyncio loop and run self.run().
        This allows the main ROS process to remain responsive while we do async IO.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self.run())
        finally:
            self._loop = None
            loop.close()

    # ---------- small queue helpers ----------

    def _drain_queue(self, q: asyncio.Queue):
        """Best-effort drain to keep queues bounded during teardown."""
        drained = 0
        try:
            while True:
                q.get_nowait()
                drained += 1
        except QueueEmpty:
            if drained:
                logger.debug("Drained %d items from queue.", drained)

    async def _flush_queues(self, where: str):
        """Drain known queues on shutdown to free memory quickly."""
        logger.info("Queues flushed (%s).", where)
        try:
            self._drain_queue(self.audio_in_queue)
        except Exception:
            pass
        try:
            self._drain_queue(self.out_queue)
        except Exception:
            pass
        logger.debug("Queues flushed (%s).", where)

    # ---------- optional simple WAV logging ----------

    async def _ensure_log_dirs(self):
        for d in (self._log_input_dir, self._log_output_dir):
            d.mkdir(parents=True, exist_ok=True)

    async def _open_turn_logs(self):
        """Open input/output WAV files for the current turn (lazy)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._in_wav or self._out_wav:
                return
            await self._ensure_log_dirs()
            self._turn_id += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            in_path = self._log_input_dir / f"turn{self._turn_id:04d}_{ts}_in.wav"
            out_path = self._log_output_dir / f"turn{self._turn_id:04d}_{ts}_out.wav"

            # Input (16k mono, PCM16)
            self._in_wav = wave.open(str(in_path), "wb")
            self._in_wav.setnchannels(1)
            self._in_wav.setsampwidth(2)  # 16-bit
            self._in_wav.setframerate(SEND_SAMPLE_RATE)

            # Output (24k mono, PCM16)
            self._out_wav = wave.open(str(out_path), "wb")
            self._out_wav.setnchannels(1)
            self._out_wav.setsampwidth(2)
            self._out_wav.setframerate(RECEIVE_SAMPLE_RATE)

            logger.debug("Turn %04d: logging to %s and %s", self._turn_id, in_path, out_path)

    async def _close_turn_logs(self):
        """Close any open WAVs gracefully."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            try:
                if self._in_wav:
                    self._in_wav.close()
            except Exception:
                logger.exception("Error closing input WAV")
            finally:
                self._in_wav = None

            try:
                if self._out_wav:
                    self._out_wav.close()
            except Exception:
                logger.exception("Error closing output WAV")
            finally:
                self._out_wav = None

    async def _log_input_bytes(self, pcm: bytes):
        """Append input PCM to the 'in' log (optional)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._in_wav is None:
                await self._open_turn_logs()
            try:
                self._in_wav.writeframes(pcm)
            except Exception:
                logger.exception("Failed to write input audio log")

    async def _log_output_bytes(self, pcm: bytes):
        """Append output PCM to the 'out' log (optional)."""
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._out_wav is None:
                await self._open_turn_logs()
            try:
                self._out_wav.writeframes(pcm)
            except Exception:
                logger.exception("Failed to write output audio log")

    # ---------- ChatNode service helpers ----------

    def _ensure_srv_client(self):
        """
        Lazily create a ROS node + service client for CreateOrUpdateChatMessage.
        We keep this very small so AudioLoop stays decoupled from ChatNode internals.
        """
        if self._srv_node is None:
            if not rclpy.ok():
                rclpy.init()
            self._srv_node = Node("gemini_audio_loop_client")
            self._srv_client = self._srv_node.create_client(
                CreateOrUpdateChatMessage, "create_or_update_chat_message"
            )
            if not self._srv_client.wait_for_service(timeout_sec=2.0):
                logger.warning("Service 'create_or_update_chat_message' not available yet.")

    def _start_chat_worker(self):
        """Start background thread that handles CreateOrUpdateChatMessage calls."""
        if self._chat_worker and self._chat_worker.is_alive():
            return

        def _worker():
            logger.debug("Chat worker thread started.")
            while not self._stop_event.is_set():
                try:
                    req = self._chat_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                if req is None:
                    # Sentinel for shutdown
                    break

                try:
                    self._ensure_srv_client()
                    if self._srv_client is None:
                        continue

                    future = self._srv_client.call_async(req)
                    rclpy.spin_until_future_complete(
                        self._srv_node, future, timeout_sec=1.0
                    )

                    if (
                        future.done()
                        and future.result() is not None
                        and future.result().successful
                    ):
                        # keep message_id for subsequent UPDATEs
                        self._last_pib_message_id = future.result().message_id
                except Exception:
                    logger.debug(
                        "Chat worker: service call failed or timed out.", exc_info=True
                    )

            logger.debug("Chat worker thread exiting.")

        self._chat_worker = threading.Thread(target=_worker, daemon=True)
        self._chat_worker.start()

    def _start_new_stream(self, role: str):
        """
        Reset accumulation for a new logical message (role = 'user' or 'assistant').
        This ensures we create separate PIB messages for each side of the dialog.
        """
        if self._current_role is not None and self._accum_text:
            prev_is_user = self._current_role == "user"
            self._send_chat_piece(
                text_piece="",           # don’t change accumulator
                is_user=prev_is_user,
                update_db=True,
                force_flush=True,        # ignore throttle
            )

        self._current_role = role
        self._accum_text = ""
        self._last_pib_message_id = ""

    def _send_chat_piece(
    self,
    text_piece: str,
    is_user: bool,
    update_db: bool = True,
    force_flush: bool = False,
    ):
        """
        Send accumulated text to ChatNode service.

        - For user: usually throttled.
        - For assistant: typically force_flush=True → realtime.
        """
        if self._stop_event.is_set() or not self._is_listening:
            return

        # 1) Update accumulator
        if text_piece:
            self._accum_text = (self._accum_text + " " + text_piece).strip()

        if not self._accum_text:
            return

        now = time.time()

        # 2) Throttle only when NOT forced
        if not force_flush and now - self._last_srv_call_time < self._srv_call_min_interval:
            return

        self._last_srv_call_time = now

        # 3) Build request
        req = CreateOrUpdateChatMessage.Request()
        req.chat_id = self._chat_id or ""
        req.text = self._accum_text
        req.is_user = is_user
        req.update_database = update_db
        req.message_id = self._last_pib_message_id  # "" → CREATE on first call

        # 4) Queue for chat worker
        try:
            self._chat_queue.put_nowait(req)
        except queue.Full:
            logger.debug("Chat worker queue full, dropping chat update.")

    # ---------- tasks ----------

    async def _listen_from_ros(self):
        """Pulls PCM16 mono @16k from ROS topic into out_queue for the live session."""
        logger.info(f"Listening from ROS topic '{ROS_AUDIO_TOPIC}' (expect PCM16 mono @16k).")
        try:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.debug("_listen_from_ros: cancelled")
            raise

    async def send_realtime(self):
        """Feeds PCM chunks from out_queue into the Gemini live session."""
        try:
            while not self._stop_event.is_set():
                msg = await self.out_queue.get()
                # If desired, uncomment to log input audio:
                # data = msg.get("data", b"")
                # if data:
                #     await self._log_input_bytes(data)
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.debug("send_realtime: cancelled")
            raise
        except Exception:
            logger.exception("send_realtime error")

    def _log_transcriptions(self, sc):
        """
        Called for each server_content event from Gemini:
        - input_transcription  → what the user said (streaming text)
        - output_transcription → what Gemini is saying (streaming text)
        We forward slices into ChatNode so the UI updates while audio plays.
        """
        if self._stop_event.is_set() or not self._is_listening:
            return

        # User stream
        input_transcription = getattr(sc, "input_transcription", None)
        if input_transcription and getattr(input_transcription, "text", None):
            text_piece = input_transcription.text.strip()
            logger.info(f"User: {text_piece}")
            if self._current_role != "user":
                self._start_new_stream("user")
            self._send_chat_piece(
                text_piece,
                is_user=True,
                update_db=True,
                force_flush=False,
            )

        # Assistant stream
        output_transcription = getattr(sc, "output_transcription", None)
        if output_transcription and getattr(output_transcription, "text", None):
            text_piece = output_transcription.text.strip()
            logger.info(f"Gemini: {text_piece}")
            if self._current_role != "assistant":
                self._start_new_stream("assistant")
            self._send_chat_piece(
                text_piece,
                is_user=False,
                update_db=True,
                force_flush=True,
            )

    async def receive_audio(self):
        """
        Receives downstream events from Gemini:
        - resp.data → PCM audio bytes (playback)
        - resp.text → occasional text events
        - resp.server_content → transcript slices (we pipe those to ChatNode)
        The loop resets per "turn" and respects Stop immediately.
        """
        while not self._stop_event.is_set():
            try:
                # New turn → reset stream roles/accumulators so messages are separate
                self._current_role = None
                self._accum_text = ""
                self._last_pib_message_id = ""
                await self._open_turn_logs()
            except Exception as e:
                logger.error(f"Failed to open turn logs: {e}")
                continue

            turn = None
            try:
                turn = self.session.receive()
                async for resp in turn:
                    if data := getattr(resp, "data", None):
                        # PCM audio from Gemini (24k mono)
                        try:
                            self.audio_in_queue.put_nowait(data)
                            # If desired, uncomment to log output audio:
                            # await self._log_output_bytes(data)
                        except asyncio.QueueFull:
                            logger.warning("Audio input queue is full; dropping data chunk.")
                    elif text := getattr(resp, "text", None):
                        # Occasionally text responses arrive without audio; print for debugging
                        print(text, end="")

                    if sc := getattr(resp, "server_content", None):
                        self._log_transcriptions(sc)
    
                    if self._stop_event.is_set():
                        # Stop requested → break out and let run() tear down
                        break

            except Exception as e:
                logger.exception(f"Error receiving audio: {e}")

            while not self.audio_in_queue.empty():
                self.audio_in_queue.get_nowait()
            # Reset audio queue between turns (avoids stale audio carrying over)
            # self.audio_in_queue = asyncio.Queue()

    async def play_audio(self):
        """Simple PyAudio playback consumer for PCM @24k mono."""
        self.playback_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            logger.info("Playback stream opened: rate=%s, channels=%s", RECEIVE_SAMPLE_RATE, CHANNELS)
        except Exception:
            pass

        try:
            while not self._stop_event.is_set():
                pcm = await self.audio_in_queue.get()
                await asyncio.to_thread(self.playback_stream.write, pcm)
        except asyncio.CancelledError:
            logger.debug("play_audio: cancelled, closing playback stream")
            try:
                self.playback_stream.close()
            finally:
                raise
        except Exception:
            logger.exception("play_audio error")

    async def run(self):
        """
        Main async entry:
        - Opens Gemini live session (genai client)
        - Starts ROS bridge + 3 tasks (listen/send/receive/play)
        - Waits for first exception to shut down everything together
        """
        client = genai.Client(api_key=self.api_key)
        ros_started = False
        tasks = []
        try:
            # Read chat personality/description from PIB to seed the model
            successful, personality = voice_assistant_client.get_personality_from_chat(self._chat_id)
            if not successful:
                logger.error(f"no personality found for id {self._chat_id}")
            description = (
                personality.description if personality.description is not None else "You are pib, a humanoid robot."
            )

            gemini_config = dict(CONFIG)
            gemini_config["system_instruction"] = description

            # Live session
            async with client.aio.live.connect(model=MODEL, config=gemini_config) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)
                self._log_lock = asyncio.Lock()

                # Optional folders for WAV logs
                await self._ensure_log_dirs()

                # Start ROS subscriber thread that feeds PCM into out_queue
                topic = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")
                self._ros_bridge = RosAudioBridge(topic, asyncio.get_running_loop(), self.out_queue)
                self._ros_bridge.start()
                ros_started = True
                logger.info("RosAudioBridge started.")

                # Start async tasks
                tasks = [
                    asyncio.create_task(self._listen_from_ros()),
                    asyncio.create_task(self.send_realtime()),
                    asyncio.create_task(self.receive_audio()),
                    asyncio.create_task(self.play_audio()),
                ]

                # Wait until one task errors, then tear down everything
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
                for t in done:
                    if t.exception():
                        raise t.exception()

        except asyncio.CancelledError:
            logger.debug("GeminiAudioLoop.run: cancelled")
        except Exception:
            logger.exception("GeminiAudioLoop.run: unexpected error")
        finally:
            # Cancel all tasks and wait; ignore exceptions during teardown
            for t in tasks:
                t.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)

            # Stop ROS bridge if it was started
            if ros_started and self._ros_bridge:
                try:
                    self._ros_bridge.stop()
                except Exception:
                    pass
                self._ros_bridge = None

            # Close playback stream if still open
            if self.playback_stream:
                try:
                    self.playback_stream.close()
                except Exception:
                    pass

            # Drain queues and close WAVs
            try:
                await self._flush_queues("shutdown")
            except Exception:
                pass
            try:
                await self._close_turn_logs()
            except Exception:
                pass

            logger.info("GeminiAudioLoop.run: terminated")


def main():
    # Optional local entry for quick manual test (not used in ROS launch)
    loop = GeminiAudioLoop(api_key=os.getenv("GOOGLE_API_KEY", ""))
    loop.start()


if __name__ == "__main__":
    main()
