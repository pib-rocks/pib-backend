"""Live voice through the vendored Hermes Talk provider (PR-2252).

Same surface as ``GeminiAudioLoop`` — ``start(chat_id, model, idle_timeout_s)``,
``stop()``, ``is_listening``, ``submit_typed_text`` and the two listeners — so
``assistant.py`` and the Cerebra UI keep working unchanged.

Audio: ``/audio_stream`` (16 kHz mono PCM) -> ``AppendInputAudio``; the model's
``OutputAudio`` (24 kHz) goes to the same PyAudio sink the old loop used.
Transcripts are written through ChatNode's ``CreateOrUpdateChatMessage`` service,
one message per final transcript.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
import uuid

import numpy as np
import pyaudio

from datatypes.srv import CreateOrUpdateChatMessage
from voice_assistant.audio_loop import (
    CHANNELS,
    FORMAT,
    RECEIVE_SAMPLE_RATE,
    ROS_AUDIO_TOPIC,
    RosAudioBridge,
)
from voice_assistant.cloud_session import (
    CascadeResult,
    CloudSession,
    mint_session,
    report_usage,
)
from voice_assistant.talk_provider import talk_gemini_realtime as gemini
from voice_assistant.talk_provider import talk_realtime as rt

logger = logging.getLogger("TalkLiveLoop")

#: Voice name from the Gemini prebuilt set. The personality may override later.
DEFAULT_VOICE = "Puck"


def upsample_16k_to_24k(pcm: bytes) -> bytes:
    """The provider expects 24 kHz and down-samples to 16 kHz itself.

    The array already publishes 16 kHz. Feeding it raw made the provider treat it
    as 24 kHz, so the model heard 10.7 kHz slow-motion speech and transcribed
    nothing (measured on 192.168.1.168: healthy -14 dBFS input, zero events).
    """

    samples = np.frombuffer(pcm, dtype="<i2")
    if samples.size == 0:
        return pcm
    positions = np.arange(int(round(samples.size * 1.5))) / 1.5
    values = np.interp(positions, np.arange(samples.size), samples)
    return values.astype("<i2").tobytes()


#: How long a silent session may live before it is closed and settled.
DEFAULT_IDLE_TIMEOUT_S = 180.0


class TalkLiveLoop:
    """One live session, driven by the Talk provider."""

    def __init__(self, node, *, on_session_end=None) -> None:
        self._node = node
        self._on_session_end = on_session_end
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._session: gemini.GeminiRealtimeSession | None = None
        self._session_id: str | None = None
        self._chat_id: str = ""
        self._listening = False
        self._speaking = False
        self._idle_timeout_s = DEFAULT_IDLE_TIMEOUT_S
        self._last_activity = time.monotonic()
        self._speaking_listener = None
        self._action_announcer = None
        self._chat_client = node.create_client(
            CreateOrUpdateChatMessage, "create_or_update_chat_message"
        )
        self._audio_in_queue: asyncio.Queue = asyncio.Queue()
        self._ready = asyncio.Event()
        #: Deltas of the current turn, per role, so one message is written per
        #: turn instead of one per token.
        self._pending_text: dict[bool, str] = {True: "", False: ""}

    # ---------------------------------------------------------------- surface

    @property
    def is_listening(self) -> bool:
        return self._listening

    def set_speaking_listener(self, listener) -> None:
        self._speaking_listener = listener

    def set_action_announcer(self, announcer) -> None:
        self._action_announcer = announcer

    def submit_typed_text(self, text: str) -> bool:
        """Send operator text into the running live session."""

        if not self._listening or self._loop is None or self._session is None:
            return False
        command = rt.AddInputText(item_id=f"typed-{uuid.uuid4().hex[:12]}", text=text)
        try:
            asyncio.run_coroutine_threadsafe(self._session.send([command]), self._loop)
        except Exception:
            logger.exception("Could not hand typed text to the live session")
            return False
        self._note_activity()
        return True

    def start(
        self, chat_id: str, model: str | None = None, idle_timeout_s=None
    ) -> None:
        if self._listening:
            return
        self._chat_id = chat_id or ""
        self._idle_timeout_s = float(idle_timeout_s or DEFAULT_IDLE_TIMEOUT_S)
        self._stop_event.clear()
        self._listening = True
        self._speaking = False
        self._note_activity()
        self._thread = threading.Thread(
            target=self._thread_main, args=(model,), name="TalkLiveLoop", daemon=True
        )
        self._thread.start()

    def stop(self, join_timeout: float = 5.0) -> None:
        self._stop_event.set()
        loop = self._loop
        session = self._session
        if loop is not None and session is not None:
            try:
                asyncio.run_coroutine_threadsafe(session.close(), loop)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
            self._thread = None
        self._set_speaking(False)
        self._listening = False
        self._session = None
        self._loop = None

    # -------------------------------------------------------------- internals

    def _note_activity(self) -> None:
        self._last_activity = time.monotonic()

    def _set_speaking(self, speaking: bool) -> None:
        if speaking == self._speaking:
            return
        self._speaking = speaking
        listener = self._speaking_listener
        if listener is not None:
            try:
                listener(speaking)
            except Exception:
                logger.exception("speaking listener failed")

    def _tool_definitions(self) -> tuple:
        """The MCP catalogue as provider-neutral tool definitions."""

        try:
            from voice_assistant.direct_tool_loop import mcp_tool_declarations

            return tuple(
                rt.ToolDefinition(
                    name=entry["name"],
                    description=entry.get("description", ""),
                    parameters=entry.get(
                        "parameters", {"type": "object", "properties": {}}
                    ),
                )
                for entry in mcp_tool_declarations()
            )
        except Exception:
            logger.exception("Could not read the MCP tool declarations")
            return ()

    def _run_tool(self, name: str, arguments: str) -> str:
        from voice_assistant.direct_tool_loop import execute_mcp_tool

        try:
            parsed = json.loads(arguments) if arguments else {}
        except Exception:
            parsed = {}
        try:
            outcome = execute_mcp_tool(name, parsed)
        except Exception as exc:
            logger.exception("Tool %s failed", name)
            outcome = {
                "ok": False,
                "error": {"code": "tool_failed", "message": str(exc)},
            }
        return json.dumps(outcome, ensure_ascii=False)

    def _personality_text(self) -> str:
        try:
            from pib_api_client import voice_assistant_client

            ok, personality = voice_assistant_client.get_personality_from_chat(
                self._chat_id
            )
            if ok and getattr(personality, "description", None):
                return personality.description
        except Exception:
            logger.exception("Could not read the personality; using a default")
        return "Du bist pib, ein humanoider Roboter."

    def _write_transcript(self, text: str, *, is_user: bool) -> None:
        if not text.strip() or not self._chat_id:
            return
        request = CreateOrUpdateChatMessage.Request()
        request.chat_id = self._chat_id
        request.text = text.strip()
        request.is_user = is_user
        request.update_database = True
        request.message_id = ""
        if not self._chat_client.service_is_ready():
            if not self._chat_client.wait_for_service(timeout_sec=1.0):
                logger.warning("Chat service unavailable; transcript dropped")
                return
        self._chat_client.call_async(request)

    def _thread_main(self, model: str | None) -> None:
        try:
            asyncio.run(self._session_main(model))
        except Exception:
            logger.exception("Talk live loop ended with an error")
        finally:
            self._listening = False
            self._set_speaking(False)
            callback = self._on_session_end
            if callback is not None:
                try:
                    callback("closed")
                except Exception:
                    logger.exception("on_session_end callback failed")

    async def _session_main(self, model: str | None) -> None:
        self._loop = asyncio.get_running_loop()
        wire_model = model or "gemini-3.8-live"
        minted = mint_session(wire_model)
        if isinstance(minted, CascadeResult):
            logger.warning("pib.Cloud declined the live session: %s", minted.reason)
            return
        if not isinstance(minted, CloudSession):
            logger.error("pib.Cloud returned no usable session")
            return

        self._session_id = minted.session_id
        session = gemini.GeminiRealtimeSession(
            auth_token=minted.ephemeral_token, auth_source="pib-cloud"
        )
        self._session = session
        tools = await asyncio.to_thread(self._tool_definitions)
        setup = rt.SessionSetup(
            model=wire_model,
            voice=DEFAULT_VOICE,
            instructions=self._personality_text(),
            tools=tools,
        )
        try:
            await session.connect(setup)
        except Exception:
            logger.exception("Could not open the Talk session")
            return

        bridge = RosAudioBridge(ROS_AUDIO_TOPIC, self._loop, self._audio_in_queue)
        bridge.start()
        tasks = [
            asyncio.create_task(self._uplink(session)),
            asyncio.create_task(self._downlink(session)),
            asyncio.create_task(self._idle_watchdog()),
        ]
        try:
            await self._stop_waiter()
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            bridge.stop()
            try:
                await session.close()
            except Exception:
                pass
            if self._session_id:
                try:
                    report_usage(self._session_id, None)
                except Exception:
                    logger.exception("Could not settle the pib.Cloud session")

    async def _stop_waiter(self) -> None:
        while not self._stop_event.is_set():
            await asyncio.sleep(0.2)

    async def _uplink(self, session) -> None:
        # connect() returns while the handshake is still in flight; the wire
        # answers 1007-class errors ("not connected") to audio sent too early.
        deadline = time.monotonic() + 10.0
        while (
            session.state is not rt.SessionState.CONNECTED
            and not self._stop_event.is_set()
        ):
            if time.monotonic() > deadline:
                logger.error("Live session did not become ready; uplink not started")
                return
            await asyncio.sleep(0.05)
        sent = 0
        logger.info("uplink started")
        while not self._stop_event.is_set():
            message = await self._audio_in_queue.get()
            data = message.get("data", b"")
            sent += 1
            if sent % 100 == 0:
                logger.info("uplink sent %s chunks (%s bytes)", sent, len(data))
            if not data:
                continue
            self._note_activity()
            try:
                await session.send(
                    [rt.AppendInputAudio(data=upsample_16k_to_24k(data))]
                )
            except Exception:
                logger.exception("Uploading a microphone chunk failed")
                return

    async def _downlink(self, session) -> None:
        stream = None
        try:
            async for event in session:
                self._note_activity()
                if not isinstance(event, rt.OutputAudio):
                    logger.info("live event: %s", type(event).__name__)
                if isinstance(event, rt.SessionReady):
                    self._ready.set()
                if isinstance(event, rt.OutputAudio):
                    if stream is None:
                        stream = await asyncio.to_thread(self._open_playback)
                    self._set_speaking(True)
                    await asyncio.to_thread(stream.write, event.data)
                elif isinstance(event, rt.Transcript):
                    is_user = event.role is rt.TranscriptRole.USER
                    piece = event.text.strip()
                    if piece:
                        self._pending_text[is_user] = (
                            self._pending_text[is_user] + " " + piece
                        ).strip()
                    if event.final:
                        text = self._pending_text[is_user]
                        self._pending_text[is_user] = ""
                        self._write_transcript(text, is_user=is_user)
                    if not is_user:
                        self._set_speaking(event.final is False)
                elif isinstance(event, rt.FunctionCall):
                    output = await asyncio.to_thread(
                        self._run_tool, event.name, event.arguments
                    )
                    await session.send(
                        [rt.SubmitToolResult(call_id=event.call_id, output=output)]
                    )
                elif isinstance(event, rt.OutputInterrupted):
                    self._set_speaking(False)
                elif isinstance(event, rt.OutputTurnCompleted):
                    self._set_speaking(False)
        finally:
            self._set_speaking(False)
            if stream is not None:
                await asyncio.to_thread(stream.close)

    def _open_playback(self):
        return pyaudio.PyAudio().open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )

    async def _idle_watchdog(self) -> None:
        while not self._stop_event.is_set():
            await asyncio.sleep(0.5)
            if time.monotonic() - self._last_activity > self._idle_timeout_s:
                logger.info(
                    "Live session idle for %.0fs; stopping so it does not keep billing.",
                    self._idle_timeout_s,
                )
                self._stop_event.set()
                return
