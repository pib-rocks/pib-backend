import asyncio
import os
import logging
import threading
from typing import Any, Optional
import asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int16MultiArray

import pyaudio
from google import genai

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
CHANNELS = 1                 # send mono to Gemini
SEND_SAMPLE_RATE = 16000     # recorder publishes 16 kHz mono
RECEIVE_SAMPLE_RATE = 24000  # model replies at 24 kHz (your speaker supports it)
CHUNK_SIZE = 1024

MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = {"response_modalities": ["AUDIO"]}

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

    def __init__(self, topic: str, loop: asyncio.AbstractEventLoop, out_queue: asyncio.Queue):
        self._topic = topic
        self._loop = loop
        self._q = out_queue
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._started_evt = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, name="RosAudioBridge", daemon=True)
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
                # If your process hasn't called rclpy.init(), you can do it here.
                # In your launch, rclpy is already initialized, so we skip.
                pass

            class _Node(Node):
                def __init__(self, topic, loop, q, stop_evt):
                    super().__init__("ros_audio_bridge")
                    self._loop = loop
                    self._q = q
                    self._stop_evt = stop_evt
                    self._subs = []
                    self._subs.append(self.create_subscription(
                        Int16MultiArray, topic, self._cb_int16, 10
                    ))
                    if AudioData is not None:
                        self._subs.append(self.create_subscription(
                            AudioData, topic, self._cb_bytes, 10
                        ))
                    self.get_logger().info(f"Subscribed to '{topic}' for PCM16 mono @16k")

                def _enqueue(self, payload: bytes):
                    if self._stop_evt.is_set():
                        return
                    async def _put():
                        await self._q.put({"data": payload, "mime_type": "audio/pcm"})
                    try:
                        # bounded backpressure — avoid QueueFull storms
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

            node = _Node(self._topic, self._loop, self._q, self._stop_evt)
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

        # Initialized in run()
        self.audio_in_queue: asyncio.Queue[bytes]
        self.out_queue: asyncio.Queue[dict[str, Any]]
        self.session = None
        self.audio_stream = None
        self.playback_stream = None
        self.api_key = api_key
        
        # ROS bridge handle
        self._ros_bridge: Optional[RosAudioBridge] = None

    @property
    def is_listening(self) -> bool:
        return self._is_listening

    def start(self) -> None:
        """Start the Gemini audio loop in a background thread."""
        if self._is_listening:
            logger.info("GeminiAudioLoop is already running.")
            return

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
        if self._thread:
            self._thread.join(timeout=join_timeout)
        self._is_listening = False
        self._thread = None
        logger.info("GeminiAudioLoop: stopped")

    def _run_wrapper(self):
        try:
            asyncio.run(self.run())
        except Exception:
            logger.exception("GeminiAudioLoop error in thread")

    # — tasks —
    async def _listen_from_ros(self):
        """Bridge thread pushes data to out_queue; we just idle until stop."""
        logger.info(f"Listening from ROS topic '{ROS_AUDIO_TOPIC}' (expect PCM16 mono @16k).")
        try:
            while not self._stop_event.is_set():
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.info("_listen_from_ros: cancelled")
            raise

    async def send_realtime(self):
        try:
            while not self._stop_event.is_set():
                msg = await self.out_queue.get()
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.info("send_realtime: cancelled")
            raise
        except Exception:
            logger.exception("send_realtime error")

    async def receive_audio(self):
        while not self._stop_event.is_set():
            turn = self.session.receive()
            async for resp in turn:
                if data := getattr(resp, "data", None):
                    self.audio_in_queue.put_nowait(data)
                elif text := getattr(resp, "text", None):
                    print(text, end="")
            # flush on interruptions
            while not self.audio_in_queue.empty():
                self.audio_in_queue.get_nowait()
    async def play_audio(self):
        # Output at 24 kHz mono (your speaker supports this)
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
            logger.info("play_audio: cancelled, closing playback stream")
            try:
                self.playback_stream.close()
            finally:
                raise
        except Exception:
            logger.exception("play_audio error")

    async def run(self):
        client = genai.Client(api_key=self.api_key)
        ros_started = False
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()  # playback side
                self.out_queue = asyncio.Queue() #ROS → Gemini side

                # start ROS bridge (topic name can be env-override if you like)
                topic = os.getenv("ROS_AUDIO_TOPIC", "audio_stream")
                self._ros_bridge = RosAudioBridge(topic, asyncio.get_running_loop(), self.out_queue)
                self._ros_bridge.start()
                logger.info("RosAudioBridge started.")

                tasks = [
                    asyncio.create_task(self._listen_from_ros()),   # idle; bridge pushes into out_queue
                    asyncio.create_task(self.send_realtime()),      # forwards to Gemini live session
                    asyncio.create_task(self.receive_audio()),      # model's audio/text back
                    asyncio.create_task(self.play_audio()),         # play 24k mono
                ]


                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
                for t in done:
                    if t.exception():
                        raise t.exception()

        except asyncio.CancelledError:
            logger.info("GeminiAudioLoop.run: cancelled")
        except Exception:
            logger.exception("GeminiAudioLoop.run: unexpected error")
        finally:
            for t in locals().get("tasks", []):
                t.cancel()
            await asyncio.gather(*locals().get("tasks", []), return_exceptions=True)

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

            logger.info("GeminiAudioLoop.run: terminated")