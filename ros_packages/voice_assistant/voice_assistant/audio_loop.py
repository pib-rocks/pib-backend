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

# service API to ChatNode
from datatypes.srv import CreateOrUpdateChatMessage

# NEW: to catch concurrency errors from websockets
from websockets.exceptions import ConcurrencyError

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
CHANNELS = 1  # send mono to Gemini
SEND_SAMPLE_RATE = 16000  # recorder publishes 16 kHz mono
RECEIVE_SAMPLE_RATE = 24000  # model replies at 24 kHz (your speaker supports it)
CHUNK_SIZE = 1024

MODEL = "gemini-2.5-flash-native-audio-preview-09-2025"
CONFIG = {
    "response_modalities": ["AUDIO"],
    "input_audio_transcription": {},
    "output_audio_transcription": {},
}
# Topic name (ROS)
ROS_AUDIO_TOPIC = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")


# ——————————————————————————————————————————
#         ROS subscriber bridge (thread)
# ——————————————————————————————————————————
class RosAudioBridge:
    """
    Subscribes to a ROS topic carrying PCM16 mono @16k (Int16MultiArray or AudioData)
    and forwards chunks into an asyncio.Queue that send_realtime() consumes.
    """

    def __init__(
        self, topic: str, loop: asyncio.AbstractEventLoop, out_queue: asyncio.Queue
    ):
        self._topic = topic
        self._loop = loop
        self._out_queue = out_queue
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._started_evt = threading.Event()

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name="RosAudioBridge", daemon=True
        )
        self._thread.start()
        self._started_evt.wait(timeout=3.0)

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _run(self):
        try:
            # optional support for audio_common_msgs/AudioData (bytes)
            AudioData = None
            try:
                from audio_common_msgs.msg import AudioData as _AudioData  # type: ignore
                AudioData = _AudioData
            except Exception:
                pass

            # IMPORTANT: reuse existing ROS context if already init'd by the launcher
            if not rclpy.ok():
                pass

            class _Node(Node):
                def __init__(self, topic, loop, queue, stop_evt):
                    super().__init__("ros_audio_bridge")
                    self._loop = loop
                    self._queue = queue
                    self._stop_evt = stop_evt
                    self._subs = []
                    self._subs.append(
                        self.create_subscription(
                            Int16MultiArray, topic, self._cb_int16, 10
                        )
                    )
                    if AudioData is not None:
                        self._subs.append(
                            self.create_subscription(
                                AudioData, topic, self._cb_bytes, 10
                            )
                        )
                    self.get_logger().info(
                        f"Subscribed to '{topic}' for PCM16 mono @16k"
                    )

                def _enqueue(self, payload: bytes):
                    if self._stop_evt.is_set():
                        return

                    async def _put():
                        await self._queue.put(
                            {"data": payload, "mime_type": "audio/pcm"}
                        )

                    try:
                        fut = asyncio.run_coroutine_threadsafe(_put(), self._loop)
                        fut.result(timeout=0.25)
                    except Exception:
                        # drop late if sender is congested
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
    def __init__(self, api_key: str = "") -> None:
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._is_listening = False
        self._chat_id = None

        # Initialized in run()
        self.audio_in_queue: asyncio.Queue[bytes]
        self.out_queue: asyncio.Queue[dict[str, Any]]
        self.session = None
        self.audio_stream = None
        self.playback_stream = None
        self.api_key = api_key

        # ROS bridge handle
        self._ros_bridge: Optional[RosAudioBridge] = None

        # Logging / turns
        self._turn_id = 0
        self._in_wav: Optional[wave.Wave_write] = None
        self._out_wav: Optional[wave.Wave_write] = None
        self._log_lock: Optional[asyncio.Lock] = None
        self._log_input_dir = Path(os.getenv("AUDIO_LOG_INPUT_DIR", "input"))
        self._log_output_dir = Path(os.getenv("AUDIO_LOG_OUTPUT_DIR", "output"))

        # ---------- service client state ----------
        self._srv_node: Optional[Node] = None
        self._srv_client = None
        self._accum_text: str = ""
        self._last_pib_message_id: str = ""
        self._current_role: Optional[str] = None  # "user" | "assistant" | None

    @property
    def is_listening(self) -> bool:
        return self._is_listening

    def start(self, chat_id) -> None:
        """Start the Gemini audio loop in a background thread."""
        # Don't start if a previous thread is still alive
        if self._thread is not None and self._thread.is_alive():
            logger.warning("GeminiAudioLoop thread is still running; not starting a new one.")
            return
        if self._is_listening:
            logger.info("GeminiAudioLoop is already running.")
            return

        self._chat_id = chat_id
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_wrapper, daemon=True)
        self._thread.start()
        self._is_listening = True
        logger.info("GeminiAudioLoop: started")

    def stop(self, join_timeout: float = 1.0) -> None:
        """Signal the loop to stop and join the thread."""
        if not self._is_listening:
            logger.info("GeminiAudioLoop is not running.")
            return

        self._stop_event.set()
        # reset accumulated text so we don't accidentally reuse it
        self._accum_text = ""
        self._last_pib_message_id = ""
        self._current_role = None

        # IMPORTANT: fully join the thread so the old session is completely torn down
        if self._thread:
            self._thread.join()  # block until run() finished
        self._is_listening = False
        self._thread = None
        logger.info("GeminiAudioLoop: stopped")

    def _run_wrapper(self):
        try:
            asyncio.run(self.run())
        except Exception:
            logger.exception("GeminiAudioLoop error in thread")

    # ——— Queue utils ———
    def _drain_queue(self, q: asyncio.Queue):
        drained = 0
        try:
            while True:
                q.get_nowait()
                drained += 1
        except QueueEmpty:
            if drained:
                logger.debug("Drained %d items from queue.", drained)

    async def _flush_queues(self, where: str):
        try:
            self._drain_queue(self.audio_in_queue)
        except Exception:
            pass
        try:
            self._drain_queue(self.out_queue)
        except Exception:
            pass
        logger.debug("Queues flushed (%s).", where)

    # ——— Turn log helpers ———
    async def _ensure_log_dirs(self):
        for d in (self._log_input_dir, self._log_output_dir):
            d.mkdir(parents=True, exist_ok=True)

    async def _open_turn_logs(self):
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._in_wav or self._out_wav:
                return
            await self._ensure_log_dirs()
            self._turn_id += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # ms precision
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

            logger.debug(
                "Turn %04d: logging to %s and %s", self._turn_id, in_path, out_path
            )

    async def _close_turn_logs(self):
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
        if self._log_lock is None:
            self._log_lock = asyncio.Lock()
        async with self._log_lock:
            if self._out_wav is None:
                await self._open_turn_logs()
            try:
                self._out_wav.writeframes(pcm)
            except Exception:
                logger.exception("Failed to write output audio log")

    # ---------- service client helpers ----------

    def _ensure_srv_client(self):
        if self._srv_node is None:
            if not rclpy.ok():
                rclpy.init()
            self._srv_node = Node("gemini_audio_loop_client")
            self._srv_client = self._srv_node.create_client(
                CreateOrUpdateChatMessage, "create_or_update_chat_message"
            )
            if not self._srv_client.wait_for_service(timeout_sec=2.0):
                logger.warning("Service 'create_or_update_chat_message' not available yet.")

    def _start_new_stream(self, role: str):
        # role is "user" or "assistant"
        self._current_role = role
        self._accum_text = ""
        self._last_pib_message_id = ""

    def _send_chat_piece(self, text_piece: str, is_user: bool, update_db: bool = True):
        """Send (accumulated) text to ChatNode service. First call → CREATE, then UPDATE."""
        # If we've been stopped (button pressed), do NOT send anything more.
        if self._stop_event.is_set() or not self._is_listening:
            return
        if not text_piece:
            return

        self._ensure_srv_client()

        # accumulate and normalize spacing
        self._accum_text = (self._accum_text + " " + text_piece).strip()

        req = CreateOrUpdateChatMessage.Request()
        req.chat_id = self._chat_id or ""
        req.text = self._accum_text  # pass FULL text
        req.is_user = is_user
        req.update_database = update_db
        req.message_id = self._last_pib_message_id  # "" → CREATE

        future = self._srv_client.call_async(req)
        # Spin until the service returns (simple and safe here)
        rclpy.spin_until_future_complete(self._srv_node, future, timeout_sec=2.0)
        if future.done() and future.result() is not None and future.result().successful:
            self._last_pib_message_id = future.result().message_id
        else:
            logger.debug("CreateOrUpdateChatMessage call failed or timed out.")

    # — tasks —
    async def _listen_from_ros(self):
        logger.info(f"Listening from ROS topic '{ROS_AUDIO_TOPIC}' (expect PCM16 mono @16k).")
        try:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.debug("_listen_from_ros: cancelled")
            raise

    async def send_realtime(self):
        try:
            while not self._stop_event.is_set():
                msg = await self.out_queue.get()
                data = msg.get("data", b"")
                if data:
                    await self._log_input_bytes(data)
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.debug("send_realtime: cancelled")
            raise
        except Exception:
            logger.exception("send_realtime error")

    def _log_transcriptions(self, sc):
        """Forward streaming transcripts to ChatNode via service (+ log)."""
        # If we've been stopped, ignore any further transcriptions.
        if self._stop_event.is_set() or not self._is_listening:
            return

        # user stream
        it = getattr(sc, "input_transcription", None)
        if it and getattr(it, "text", None):
            text_piece = it.text.strip()
            logger.info(f"User: {text_piece}")
            if self._current_role != "user":
                self._start_new_stream("user")
            self._send_chat_piece(text_piece, is_user=True, update_db=True)

        # assistant stream
        ot = getattr(sc, "output_transcription", None)
        if ot and getattr(ot, "text", None):
            text_piece = ot.text.strip()
            logger.info(f"Gemini: {text_piece}")
            if self._current_role != "assistant":
                self._start_new_stream("assistant")
            self._send_chat_piece(text_piece, is_user=False, update_db=True)

    async def receive_audio(self):
        while not self._stop_event.is_set():
            try:
                # New turn → reset stream roles/accumulators so user and assistant get separate messages
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
                    # If stop was requested while we are still receiving, close this generator
                    if self._stop_event.is_set():
                        if hasattr(turn, "aclose"):
                            try:
                                await turn.aclose()
                            except Exception:
                                pass
                        break

                    if sc := getattr(resp, "server_content", None):
                        self._log_transcriptions(sc)

                    if data := getattr(resp, "data", None):
                        try:
                            self.audio_in_queue.put_nowait(data)
                            await self._log_output_bytes(data)
                        except asyncio.QueueFull:
                            logger.warning("Audio input queue is full; dropping data chunk.")
                    elif text := getattr(resp, "text", None):
                        print(text, end="")
            except ConcurrencyError as e:
                # This is the error you saw: break out of the loop and let run() clean up.
                logger.error(f"ConcurrencyError in receive_audio: {e}")
                break
            except Exception as e:
                logger.exception(f"Error receiving audio: {e}")

            if self._stop_event.is_set():
                # Stop requested: break outer loop as well
                break

            # Optional: reset queue if needed between turns
            self.audio_in_queue = asyncio.Queue()

    async def play_audio(self):
        self.playback_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        try:
            logger.info(
                "Playback stream opened: rate=%s, channels=%s",
                RECEIVE_SAMPLE_RATE,
                CHANNELS,
            )
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
        client = genai.Client(api_key=self.api_key)
        ros_started = False
        tasks = []
        try:
            successful, personality = voice_assistant_client.get_personality_from_chat(
                self._chat_id
            )
            if not successful:
                logger.error(f"no personality found for id {self._chat_id}")
            description = (
                personality.description
                if personality.description is not None
                else "You are pib, a humanoid robot."
            )

            gemini_config = dict(CONFIG)
            gemini_config["system_instruction"] = description

            async with client.aio.live.connect(
                model=MODEL, config=gemini_config
            ) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue()
                self._log_lock = asyncio.Lock()

                await self._ensure_log_dirs()

                # start ROS bridge
                topic = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")
                self._ros_bridge = RosAudioBridge(
                    topic, asyncio.get_running_loop(), self.out_queue
                )
                self._ros_bridge.start()
                ros_started = True
                logger.info("RosAudioBridge started.")

                tasks = [
                    asyncio.create_task(self._listen_from_ros()),
                    asyncio.create_task(self.send_realtime()),
                    asyncio.create_task(self.receive_audio()),
                    asyncio.create_task(self.play_audio()),
                ]

                done, pending = await asyncio.wait(
                    tasks, return_when=asyncio.FIRST_EXCEPTION
                )
                for t in done:
                    if t.exception():
                        raise t.exception()

        except asyncio.CancelledError:
            logger.debug("GeminiAudioLoop.run: cancelled")
        except Exception:
            logger.exception("GeminiAudioLoop.run: unexpected error")
        finally:
            for t in tasks:
                t.cancel()
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)

            if ros_started and self._ros_bridge:
                try:
                    self._ros_bridge.stop()
                except Exception:
                    pass
                self._ros_bridge = None

            if self.playback_stream:
                try:
                    self.playback_stream.close()
                except Exception:
                    pass

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
    loop = GeminiAudioLoop(api_key=os.getenv("GOOGLE_API_KEY", ""))
    loop.start()


if __name__ == "__main__":
    main()
