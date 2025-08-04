from collections import deque
from typing import Any, Callable, Optional
import asyncio
import threading
import traceback
import logging

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.client import Client
from rclpy.task import Future

import pyaudio
from google import genai
from google.genai import types

from datatypes.action import Chat, RecordAudio, RunProgram
from datatypes.msg import VoiceAssistantState, ChatIsListening
from datatypes.srv import (
    SetVoiceAssistantState,
    GetVoiceAssistantState,
    ClearPlaybackQueue,
    PlayAudioFromFile,
    PlayAudioFromSpeech,
    GetChatIsListening,
    SendChatMessage,
)
from pib_api_client.voice_assistant_client import Personality, voice_assistant_client
from sts import AudioLoop  # merged sts AudioLoop

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("VoiceAssistant")

# AudioLoop parameters will live in sts.AudioLoop

START_SIGNAL_FILE = "start.wav"
STOP_SIGNAL_FILE = "stop.wav"
MAX_SILENT_SECONDS_BEFORE = 8.0

class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__("voice_assistant")
        # ROS state
        self.cycle = 0
        self.state = VoiceAssistantState()
        self.state.turned_on = False
        self.state.chat_id = ""
        self.turning_off = False
        self.personality: Optional[Personality] = None
        self.waiting_for_transcribed_text = False
        self.code_visual_queue: deque[str] = deque()
        self.final_chat_response_received = False
        self.is_executing_program = False
        # Action/Service clients and publishers
        self.chat_client = ActionClient(self, Chat, "chat"); self.chat_client.wait_for_server()
        self.record_audio_client = ActionClient(self, RecordAudio, "record_audio"); self.record_audio_client.wait_for_server()
        self.play_from_file = self.create_client(PlayAudioFromFile, "play_audio_from_file"); self.play_from_file.wait_for_service()
        self.play_from_speech = self.create_client(PlayAudioFromSpeech, "play_audio_from_speech"); self.play_from_speech.wait_for_service()
        self.clear_queue_cli = self.create_client(ClearPlaybackQueue, "clear_playback_queue"); self.clear_queue_cli.wait_for_service()
        self.run_prog_client = ActionClient(self, RunProgram, "run_program"); self.run_prog_client.wait_for_server()
        self.voice_state_pub = self.create_publisher(VoiceAssistantState, "voice_assistant_state", 10)
        self.listen_pub = self.create_publisher(ChatIsListening, "chat_is_listening", 10)
        Service(self, SetVoiceAssistantState, "set_voice_assistant_state", self.set_state_cb)
        Service(self, GetVoiceAssistantState, "get_voice_assistant_state", self.get_state_cb)
        Service(self, GetChatIsListening, "get_chat_is_listening", self.get_listening_cb)
        Service(self, SendChatMessage, "send_chat_message", self.send_chat_cb)

        # Setup background asyncio loop for AudioLoop
        self.audio_loop = AudioLoop()
        self.audio_loop_future = None
        self.audio_loop_loop = asyncio.new_event_loop()
        t = threading.Thread(target=lambda: (asyncio.set_event_loop(self.audio_loop_loop), self.audio_loop_loop.run_forever()), daemon=True)
        t.start()
        self.get_logger().info("VoiceAssistantNode initialized")

    def start_audio_loop(self):
        if not self.audio_loop_future or self.audio_loop_future.done():
            logger.info("Starting integrated Gemini/ROS audio loop")
            self.audio_loop_future = asyncio.run_coroutine_threadsafe(
                self.audio_loop.run(), self.audio_loop_loop
            )
            self.audio_loop_future.add_done_callback(self.on_audio_loop_done)

    def on_audio_loop_done(self, fut: asyncio.Future):
        if fut.cancelled():
            logger.warn("AudioLoop cancelled")
        elif exc := fut.exception():
            logger.error(f"AudioLoop crashed: {exc!r}")
        else:
            logger.info("AudioLoop exited normally")

    # Example integration point: on START_SIGNAL play
    def on_start_signal_played(self) -> None:
        if self.personality and "gemini" in self.personality.assistant_model.api_name.lower():
            self.start_audio_loop()
        else:
            # existing record_audio fallback
            pass
        # publish listening
        self.set_listening(self.state.chat_id, True)

    # Services and state management omitted for brevity...
    def set_state_cb(self, req, resp): ...
    def get_state_cb(self, req, resp): ...
    def get_listening_cb(self, req, resp): ...
    def send_chat_cb(self, req, resp): ...
    def set_listening(self, chat_id: str, listening: bool): ...

def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cancel audio loop
        if node.audio_loop_future:
            node.audio_loop_future.cancel()
            node.audio_loop_loop.call_soon_threadsafe(node.audio_loop_loop.stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
