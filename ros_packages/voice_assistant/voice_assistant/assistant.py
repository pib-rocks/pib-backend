from collections import deque
from typing import Any, Callable, Optional
import asyncio

import rclpy
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
from pib_api_client import voice_assistant_client
from pib_api_client.voice_assistant_client import Personality
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.task import Future
from voice_assistant import START_SIGNAL_FILE, STOP_SIGNAL_FILE
# from voice_assistant.audio_loop import GeminiAudioLoop

MAX_SILENT_SECONDS_BEFORE = 8.0

import asyncio
import traceback
import logging
import pyaudio
from google import genai
from google.genai import types
# ——— Configure logging ———
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("GeminiAudioLoop")

pya = pyaudio.PyAudio()

FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024
MODEL = "gemini-2.5-flash-preview-native-audio-dialog"
CONFIG = types.LiveConnectConfig(response_modalities=["AUDIO"])

class GeminiAudioLoop:
    def __init__(self):
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.audio_stream = None

    async def listen_audio(self):
        mic_info = pya.get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=SEND_RATE,
            input=True,
            input_device_index=mic_info["index"],
            frames_per_buffer=CHUNK,
        )
        kwargs = {"exception_on_overflow": False}
        try:
            while True:
                data = await asyncio.to_thread(self.audio_stream.read, CHUNK, **kwargs)
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
        except asyncio.CancelledError:
            logger.info("listen_audio: cancelled, closing stream")
            self.audio_stream.close()
            raise
        except Exception as e:
            logger.exception(f"listen_audio error: {e}")

    async def send_realtime(self):
        try:
            while True:
                msg = await self.out_queue.get()
                await self.session.send_realtime_input(audio=msg)
        except asyncio.CancelledError:
            logger.info("send_realtime: cancelled")
            raise
        except Exception as e:
            logger.exception(f"send_realtime error: {e}")

    async def receive_audio(self):
        try:
            while True:
                turn = self.session.receive()
                async for resp in turn:
                    if data := resp.data:
                        self.audio_in_queue.put_nowait(data)
                    elif text := resp.text:
                        print(text, end="")
                # clear queue on interruptions
                while not self.audio_in_queue.empty():
                    self.audio_in_queue.get_nowait()
        except asyncio.CancelledError:
            logger.info("receive_audio: cancelled")
            raise
        except Exception as e:
            logger.exception(f"receive_audio error: {e}")

    async def play_audio(self):
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECV_RATE,
            output=True,
        )
        try:
            while True:
                pcm = await self.audio_in_queue.get()
                await asyncio.to_thread(stream.write, pcm)
        except asyncio.CancelledError:
            logger.info("play_audio: cancelled, closing playback stream")
            stream.close()
            raise
        except Exception as e:
            logger.exception(f"play_audio error: {e}")

    async def close(self):
        self.audio_stream.close()

    async def run(self):
        client = genai.Client(api_key="")  # or let it pick up ADC
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)

                # start all four subtasks and wait for any to error/cancel
                async with asyncio.TaskGroup() as tg:
                    tg.create_task(self.listen_audio())
                    tg.create_task(self.send_realtime())
                    tg.create_task(self.receive_audio())
                    tg.create_task(self.play_audio())
        except asyncio.CancelledError:
            logger.info("GeminiAudioLoop.run: cancelled, shutting down")
            pass
        except Exception as eg:
            logger.exception("GeminiAudioLoop.run: unexpected error")
            traceback.print_exception(eg)
        finally:
            if self.audio_stream:
                self.audio_stream.close()
            logger.info("GeminiAudioLoop.run: terminated")

import asyncio
from contextlib import suppress

import rclpy
from voice_assistant.assistant import VoiceAssistantNode

class VoiceAssistantNode(Node):

    def __init__(self):

        super().__init__("voice_assistant")

        # ——— Set up a standalone asyncio loop in its own thread ———
        import threading, asyncio
        self.asyncio_loop = asyncio.new_event_loop()
        t = threading.Thread(
            target=lambda: (asyncio.set_event_loop(self.asyncio_loop),
                            self.asyncio_loop.run_forever()),
        # ————————————————————————————————————————————————————————
            daemon=True
        )
        t.start()

        # state -------------------------------------------------------------------------

        # a counter for indicating the index of the current on-off-cycle
        self.cycle: int = 0
        # contains information on whether the va is turned on (and what the active chat is)
        self.state: VoiceAssistantState = VoiceAssistantState()
        # indicates if the va is turned on or off
        self.state.turned_on = False
        # id of the active chat (may be an arbitrary value, if the va is turned off)
        self.state.chat_id = ""
        # indicates if the voice_assistant is currently turning off
        self.turning_off = False
        # the personality associated with the active chat
        self.personality: Optional[Personality] = None
        # calling this function stops audio-recording
        self.stop_recording: Callable[[], None] = lambda: None
        # maps a chat-id to a function that can be used to stop receiving llm-responses
        self.chat_id_to_stop_chat: dict[str, Callable[[], None]] = {}
        # maps a chat-id to the listening status of the respective chat
        self.chat_id_to_is_listening: dict[str, bool] = {}
        # indicates, whether audio was recorded and va is currently awaitng the transcription
        self.waiting_for_transcribed_text = False
        # programs (in for of visual-code) received from the chat-server are buffered here until the current program finished executing
        self.code_visual_queue: deque[str] = deque()
        # indicates whether the final response (code or sentence) in the current request/response cycle of the active chat was already received
        self.final_chat_response_received = False
        # calling this function stops the current program
        self.stop_program_execution: Callable[[], None] = lambda: None
        # indicates if a program is currently executing
        self.is_executing_program = False
        # Gemini audio loop management
        self.gemini_audio_loop = GeminiAudioLoop()
        self.gemini_task = None

        # services ----------------------------------------------------------------------

        # Service for setting VoiceAssistantState
        self.set_voice_assistant_service: Service = self.create_service(
            SetVoiceAssistantState,
            "set_voice_assistant_state",
            self.set_voice_assistant_state,
        )

        # Service for getting current VoiceAssistantState
        self.get_voice_assistant_service: Service = self.create_service(
            GetVoiceAssistantState,
            "get_voice_assistant_state",
            self.get_voice_assistant_state,
        )

        # Service for getting the listening status of a chat
        self.get_chat_is_listening_service: Service = self.create_service(
            GetChatIsListening, "get_chat_is_listening", self.get_chat_is_listening
        )

        # Service that allows clients to send chat messages
        self.send_chat_message: Service = self.create_service(
            SendChatMessage, "send_chat_message", self.send_chat_message
        )

        # publishers --------------------------------------------------------------------

        # Publisher for VoiceAssistantState
        self.voice_assistant_state_publisher: Publisher = self.create_publisher(
            VoiceAssistantState, "voice_assistant_state", 10
        )

        # Publisher for ChatIsListening
        self.chat_is_listening_publisher: Publisher = self.create_publisher(
            ChatIsListening, "chat_is_listening", 10
        )

        # clients -----------------------------------------------------------------------

        self.chat_client: ActionClient = ActionClient(self, Chat, "chat")
        self.chat_client.wait_for_server()

        self.record_audio_client: ActionClient = ActionClient(
            self, RecordAudio, "record_audio"
        )
        self.record_audio_client.wait_for_server()

        self.play_audio_from_file_client: Client = self.create_client(
            PlayAudioFromFile, "play_audio_from_file"
        )
        self.play_audio_from_file_client.wait_for_service()

        self.play_audio_from_speech_client: Client = self.create_client(
            PlayAudioFromSpeech, "play_audio_from_speech"
        )
        self.play_audio_from_speech_client.wait_for_service()

        self.clear_playback_queue_client: Client = self.create_client(
            ClearPlaybackQueue, "clear_playback_queue"
        )
        self.clear_playback_queue_client.wait_for_service()

        self.run_program_client: ActionClient = ActionClient(
            self, RunProgram, "run_program"
        )
        self.run_program_client.wait_for_server()

        self.get_logger().info("Now running VA")

    # client accessors ------------------------------------------------------------------

    async def _gemini_worker(self):
        await self.gemini_audio_loop.run()

    def clear_playback_queue(
        self, on_playback_queue_cleared: Callable[[], None] = None
    ):

        future = self.clear_playback_queue_client.call_async(
            ClearPlaybackQueue.Request()
        )
        future.add_done_callback(lambda _: on_playback_queue_cleared())

    def record_audio(
        self,
        max_silent_seconds_before: float,
        max_silent_seconds_after: float,
        on_stopped_recording: Callable[[], None] = None,
        on_transcribed_text_received: Callable[[str], None] = None,
    ) -> None:

        goal = RecordAudio.Goal()
        goal.max_silent_seconds_before = max_silent_seconds_before
        goal.max_silent_seconds_after = max_silent_seconds_after
        feedback_callback = (
            None if on_stopped_recording is None else lambda _: on_stopped_recording()
        )
        result_callback = (
            None
            if on_transcribed_text_received is None
            else lambda res: on_transcribed_text_received(res.transcribed_text)
        )
        future: Future = self.record_audio_client.send_goal_async(
            goal, feedback_callback
        )
        self.stop_recording = self.digest_goal_handle_future(future, result_callback)

    def chat(
        self,
        text: str,
        chat_id: str,
        generate_code: bool,
        on_sentence_received: Callable[[str, bool], None] = lambda _1, _2: None,
        on_code_visual_received: Callable[[str, bool], None] = lambda _1, _2: None,
    ) -> None:

        goal = Chat.Goal()
        goal.text = text
        goal.chat_id = chat_id
        goal.generate_code = generate_code

        def feedback_callback(msg) -> None:
            feedback: Chat.Feedback = msg.feedback
            text = feedback.text
            if feedback.text_type == Chat.Goal.TEXT_TYPE_SENTENCE:
                on_sentence_received(text, False)
            elif feedback.text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL:
                on_code_visual_received(text, False)
            else:
                raise Exception(f"unsupported text-type: {feedback.text_type}")

        def result_callback(result: Chat.Result) -> None:
            text = result.text
            if result.text_type == Chat.Goal.TEXT_TYPE_SENTENCE:
                on_sentence_received(text, True)
            elif result.text_type == Chat.Goal.TEXT_TYPE_CODE_VISUAL:
                on_code_visual_received(text, True)
            else:
                raise Exception(f"unsupported text-type: {result.text_type}")

        future: Future = self.chat_client.send_goal_async(goal, feedback_callback)
        stop_chat = self.digest_goal_handle_future(future, result_callback)
        self.chat_id_to_stop_chat[chat_id] = stop_chat

    def play_audio_from_file(
        self, filepath: str, on_stopped_playing: Callable[[], None] = None
    ) -> None:
        request = PlayAudioFromFile.Request()
        request.filepath = filepath
        request.join = on_stopped_playing is not None
        future: Future = self.play_audio_from_file_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def play_audio_from_speech(
        self,
        speech: str,
        gender: str,
        language: str,
        on_stopped_playing: Callable[[], None] = None,
    ) -> None:
        request = PlayAudioFromSpeech.Request()
        request.speech = speech
        request.gender = gender
        request.language = language
        request.join = on_stopped_playing is not None
        future: Future = self.play_audio_from_speech_client.call_async(request)
        if request.join:
            future.add_done_callback(lambda _: on_stopped_playing())

    def run_program(
        self, code_visual: str, on_stopped_executing_program: Callable[[None], None]
    ):

        goal = RunProgram.Goal()
        goal.source_type = RunProgram.Goal.SOURCE_CODE_VISUAL
        goal.source = code_visual
        result_callback = lambda _: on_stopped_executing_program()
        future: Future = self.run_program_client.send_goal_async(goal)
        self.stop_program_execution = self.digest_goal_handle_future(
            future, result_callback
        )

    # serivce callbacks -----------------------------------------------------------------

    def get_voice_assistant_state(
        self,
        _: GetVoiceAssistantState.Request,
        response: GetVoiceAssistantState.Response,
    ) -> GetVoiceAssistantState.Response:
        """callback function for 'get_voice_assistant_state' service"""
        response.voice_assistant_state = self.state
        return response

    def set_voice_assistant_state(
        self,
        request: SetVoiceAssistantState.Request,
        response: SetVoiceAssistantState.Response,
    ) -> SetVoiceAssistantState.Response:
        """callback function for 'set_voice_assistant_state' service"""
        request_state: VoiceAssistantState = request.voice_assistant_state
        successful = self.update_state(request_state.turned_on, request_state.chat_id)
        response.successful = successful
        return response

    def get_chat_is_listening(
        self, request: GetChatIsListening.Request, response: GetChatIsListening.Response
    ) -> GetChatIsListening.Response:
        """callback function for 'get_chat_is_listening' service"""
        response.listening = self.get_is_listening(request.chat_id)
        return response

    def send_chat_message(
        self, request: SendChatMessage.Request, response: SendChatMessage.Response
    ) -> SendChatMessage.Response:
        """callback function for 'send_chat_message' service"""

        # do not create a message, if chat is not listening
        if not self.get_is_listening(request.chat_id):
            return response

        # if chat is active, jump to next stage of the va-cycle
        elif request.chat_id == self.state.chat_id:
            self.set_is_listening(request.chat_id, False)
            self.play_audio_from_file(STOP_SIGNAL_FILE)
            self.stop_recording()
            self.set_is_listening(request.chat_id, False)
            self.chat(
                request.content,
                self.state.chat_id,
                True,
                self.if_cycle_not_changed(self.on_sentence_received),
                self.if_cycle_not_changed(self.on_code_visual_received),
                )

        # if not active, create messages, without playing audio etc.
        else:
            self.set_is_listening(request.chat_id, False)

            def on_sentence_received(sentence: str, is_final: bool):
                if is_final:
                    self.set_is_listening(request.chat_id, True)

            self.chat(  # TODO : there is a race condition here, that could lead to the va falsely starting to listen, when activating this chat
                request.content, request.chat_id, False, on_sentence_received
            )

        response.successful = True
        return response

    # callback cycle --------------------------------------------------------------------

    def _on_gemini_done(self, fut: asyncio.Future):
        if fut.cancelled():
            self.get_logger().warn("Gemini task was cancelled")
        elif exc := fut.exception():
            self.get_logger().error(f"GeminiAudioLoop crashed: {exc!r}")
        else:
            self.get_logger().info("GeminiAudioLoop exited normally")


    def on_start_signal_played(self) -> None:
        if self.personality and "gemini" in self.personality.assistant_model.api_name.lower():
            # schedule the GeminiAudioLoop on our background asyncio loop
            self.get_logger().info("starting GeminiAudioLoop task…")
            self.gemini_task = asyncio.run_coroutine_threadsafe(
                self._gemini_worker(), 
                self.asyncio_loop
            )
        # and tack on a callback so you’ll see errors if it immediately dies:
            self.gemini_task.add_done_callback(self._on_gemini_done)
        else:
            self.record_audio(
                MAX_SILENT_SECONDS_BEFORE,
                self.personality.pause_threshold,
                self.if_cycle_not_changed(self.on_stopped_recording),
                self.if_cycle_not_changed(self.on_transcribed_text_received),
            )
        self.set_is_listening(self.state.chat_id, True)

    def on_stopped_recording(self) -> None:
        if (
            self.personality
            and "gemini" in self.personality.assistant_model.api_name.lower()
        ):
            return
        if not self.get_is_listening(self.state.chat_id):
            return
        self.play_audio_from_file(STOP_SIGNAL_FILE)
        self.set_is_listening(self.state.chat_id, False)
        self.waiting_for_transcribed_text = True

    def on_transcribed_text_received(self, transcribed_text: str) -> None:
        if (
            self.personality
            and "gemini" in self.personality.assistant_model.api_name.lower()
        ):
            return
        if not self.waiting_for_transcribed_text:
            return
        self.waiting_for_transcribed_text = False
        self.chat(
            transcribed_text,
            self.state.chat_id,
            True,
            self.if_cycle_not_changed(self.on_sentence_received),
            self.if_cycle_not_changed(self.on_code_visual_received),
        )

    def on_sentence_received(self, sentence: str, is_final: bool) -> None:
        if not sentence:
            self.update_state(False)
            return
        self.final_chat_response_received = is_final
        on_stopped_playing = (
            self.if_cycle_not_changed(self.on_final_sentence_played)
            if is_final and not self.is_executing_program
            else None
        )
        self.play_audio_from_speech(
            sentence,
            self.personality.gender,
            self.personality.language,
            on_stopped_playing,
        )

    def on_code_visual_received(self, code_visual: str, is_final: bool) -> None:
        self.final_chat_response_received = is_final
        if self.is_executing_program:
            self.code_visual_queue.append(code_visual)
        else:
            self.run_program(
                code_visual,
                self.if_cycle_not_changed(self.on_stopped_executing_program),
            )
        self.is_executing_program = True

    def on_stopped_executing_program(self) -> None:
        if self.code_visual_queue:
            code_visual = self.code_visual_queue.pop()
            self.run_program(
                code_visual,
                self.if_cycle_not_changed(self.on_stopped_executing_program),
            )
        else:
            self.is_executing_program = False
            if self.final_chat_response_received:
                self.play_audio_from_file(
                    START_SIGNAL_FILE,
                    self.if_cycle_not_changed(self.on_start_signal_played),
                )

    def on_final_sentence_played(self) -> None:
        self.play_audio_from_file(
            START_SIGNAL_FILE, self.if_cycle_not_changed(self.on_start_signal_played)
        )

    # helper functions ------------------------------------------------------------------

    def if_cycle_not_changed(self, callback: Callable) -> Callable:
        """a decorator. the decorated callback only executes, if the cycle has not changed after its creation"""
        current_cycle = self.cycle

        def decorated_callback(*args):
            if self.cycle == current_cycle:
                callback(*args)

        return decorated_callback

    def digest_goal_handle_future(
        self, goal_handle_future: Future, callback: Callable[[Any], None] = None
    ) -> Callable[[], None]:
        """adds a result callback to the goal and returns a function, that can be used to cancel the goal"""
        if callback is not None:

            def result_callback(result_future: Future):
                result = result_future.result().result
                callback(result)

            def done_callback(goal_handle_future: Future):
                goal_handle: ClientGoalHandle = goal_handle_future.result()
                result_future: Future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)

            goal_handle_future.add_done_callback(done_callback)

        def cancel(future: Future) -> None:
            goal_handle: ClientGoalHandle = future.result()
            goal_handle.cancel_goal_async()

        return lambda: goal_handle_future.add_done_callback(cancel)

    def set_is_listening(self, chat_id: str, listening: bool) -> None:
        """updates and publishes the listening status of a chat"""
        self.chat_id_to_is_listening[chat_id] = listening

        chat_is_listening = ChatIsListening()
        chat_is_listening.listening = listening
        chat_is_listening.chat_id = chat_id

        self.chat_is_listening_publisher.publish(chat_is_listening)

    def get_is_listening(self, chat_id: str) -> bool:
        """find out, if a chat is currently listening for user input"""
        return self.chat_id_to_is_listening.get(chat_id, True)

    def stop_chat(self, chat_id: str) -> None:
        """if the chat of the provided chat-id is active, stop receiving messages from the chat"""
        stop_chat = self.chat_id_to_stop_chat.get(chat_id)
        if stop_chat is not None:
            stop_chat()

    def update_state(self, turned_on: bool, chat_id: str = "") -> bool:
        """attempts to update the internal state, and returns whether this was successful"""
        try:
            # ignore if currently turning off
            if self.turning_off:
                raise Exception("voice assistant is currently turning off")

            # ignore if activation state not changed
            elif turned_on == self.state.turned_on:
                raise Exception(
                    f"voice assistant is already turned {'on' if turned_on else 'off'}."
                )

            # deactivate voice assistant
            elif not turned_on:
                self.cycle += 1
                self.turning_off = True
                self.waiting_for_transcribed_text = False
                self.stop_recording()
                self.stop_chat(chat_id)
                if hasattr(self, "gemini_task") and self.gemini_task is not None:
                    self.gemini_task = None
                self.stop_program_execution()
                self.code_visual_queue.clear()
                self.final_chat_response_received = False
                self.is_executing_program = False

                current_chat_id = self.state.chat_id

                def on_playback_queue_cleared():
                    self.turning_off = False
                    self.set_is_listening(current_chat_id, True)
                    self.play_audio_from_file(STOP_SIGNAL_FILE)

                self.clear_playback_queue(on_playback_queue_cleared)

            # activate voice assistant
            else:
                self.stop_chat(chat_id)
                successful, self.personality = (
                    voice_assistant_client.get_personality_from_chat(chat_id)
                )
                if not successful:
                    raise Exception(
                        f"no personality with chat of id {chat_id} found..."
                    )
                self.set_is_listening(chat_id, False)
                self.play_audio_from_file(
                    START_SIGNAL_FILE,
                    self.if_cycle_not_changed(self.on_start_signal_played),
                )

            self.state.turned_on = turned_on
            self.state.chat_id = chat_id

        except Exception as e:
            self.get_logger().error(
                f"following error occured while trying to update state: {str(e)}."
            )
            return False

        self.voice_assistant_state_publisher.publish(self.state)
        return True

async def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0)
    # when rclpy.ok() becomes False, this coro returns

async def main(args=None):
    rclpy.init()
    node = VoiceAssistantNode()
    spin_task   = asyncio.create_task(ros_spin(node))
    gemini_task = asyncio.create_task(node.run_gemini())

    # Block here until ROS is shut down (Ctrl-C or programmatic shutdown)
    try:
        await spin_task
    finally:
        # Then tear down Gemini cleanly
        gemini_task.cancel()
        with suppress(asyncio.CancelledError):
            await gemini_task

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())