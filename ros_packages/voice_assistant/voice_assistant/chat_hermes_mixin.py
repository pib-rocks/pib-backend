"""Hermes streaming and cloud/local turn transport for ChatNode (PR-2120)."""

from __future__ import annotations

import os
import re
from concurrent.futures import TimeoutError as FutureTimeoutError
from queue import Empty, Queue
from typing import Any, Optional

from pib_hermes_config.local_model import iter_reply
from pib_hermes_config.channel import direct_system_prompt
from pib_hermes_config.turn_taking import next_speech_delta, speech_result_tail

from public_api_client.public_voice_client import PublicApiChatMessage
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    personality_system_prompt,
    turn_channel,
)
from pib_hermes_config.retired_direct_text import legacy_direct_text_selected
from provider_registry import PIB_CLOUD_API_NAME
from voice_assistant.degraded_chat import MODE_UNLOCKED, refusal_sentence
from voice_assistant.chat_turn import STATE_COMPLETED, STATE_RUNNING


def _chat():
    import voice_assistant.chat as mod

    return mod


class ChatHermesMixin:

    def _ensure_hermes_daemon(self) -> bool:
        """Start the warm Hermes daemon in the background if it is not up yet.

        Best-effort: a failed start never blocks ChatNode construction. Turns
        fall back to oneshot subprocess when the daemon is unreachable.
        """
        try:
            from public_api_client import hermes_daemon

            ok = hermes_daemon.ensure_daemon_running()
        except Exception as exc:
            self.get_logger().warning(
                f"hermes daemon could not be started: {exc}; "
                "turns will use oneshot subprocess fallback"
            )
            return False

        if ok:
            self.get_logger().info(
                f"hermes daemon available at {hermes_daemon.daemon_base_url()}"
            )
        else:
            self.get_logger().warning(
                "hermes daemon did not become reachable; "
                "turns will use oneshot subprocess fallback"
            )
        return ok

    def _preflight_hermes_binary(self) -> bool:
        """Report at startup whether the configured Hermes CLI actually runs.

        Without this, a robot whose hermes install is missing or not mounted looks
        healthy while every Smart personality quietly answers with the
        fallback sentence. This only logs.

        This runs the CLI instead of merely stat-ing it. A file check passed on a
        live robot whose CLI died with exit 127 on every turn, because the wrapper
        execs a venv interpreter that was not mounted into the container.
        """
        path = _chat().hermes_agent_client.hermes_bin()
        try:
            ok, detail = _chat().hermes_agent_client.probe_binary(
                timeout=_chat().HERMES_PROBE_TIMEOUT_SECONDS
            )
        except Exception as exc:
            # The probe may never be the reason the chat node fails to come up.
            ok, detail = False, f"probe raised {exc!r}"

        if ok:
            self.get_logger().info(
                f"hermes agent binary available at {path}"
                + (f" ({detail})" if detail else "")
            )
            return True

        self.get_logger().error(
            f"hermes agent preflight failed for '{path}': {detail}. "
            "Smart personalities will fall back to a "
            "canned reply. Check that the hermes CLI is installed for the pib "
            "user, that PIB_HERMES_BIN points at it, and that ~/.hermes, the "
            "wrapper and the uv-managed Python directory are all bind-mounted "
            "into the ros-voice-assistant service."
        )
        return False

    def _persist_streaming_assistant_text(
        self, chat_id: str, content: str, update_message: bool
    ) -> None:
        """Write the assistant answer accumulated so far (one message_id, many updates)."""
        self.create_chat_message(
            chat_id,
            content,
            False,
            update_message,
            True,
            accumulated=True,
        )

    def _stream_chunks_to_goal(
        self, goal_handle, chat_id: str, tokens, t0: Optional[float] = None
    ) -> tuple[Optional[str], Optional[int], str]:
        """Consume a token iterable, publishing sentence/code chunks as feedback.

        Returns (prev_text, prev_text_type, curr_text) so the caller can build _chat().Chat.Result.

        Speech feedback and _chat().Chat.Result use a delta-only contract: each payload
        carries only speakable text not yet emitted on the wire (empty final is
        valid). Persisted chat messages stay cumulative. For TTFT, the first
        non-empty token is published immediately before a terminator arrives.
        """
        if t0 is None:
            t0 = _chat().time.monotonic()

        # Regex for sentence / code chunking
        sentence_pattern = re.compile(
            r"^(?!<pib-program>)(.*?)(([^\d | ^A-Z][\.|!|\?|:])|<pib-program>)",
            re.DOTALL,
        )
        code_visual_pattern = re.compile(
            r"^<pib-program>(.*?)</pib-program>", re.DOTALL
        )

        # Current and previous text fragments for feedback + persistence
        curr_text: str = ""
        prev_text: Optional[str] = None
        prev_text_type = None
        bool_update_chat_message: bool = False  # controls create vs update
        first_chunk_emitted = False
        first_sentence_flushed = False
        # Full answer as received, kept so a clause can be spoken before the
        # sentence that contains it has finished.
        raw_answer = ""
        published_speech: list[str] = []
        # A chat message is one row that the UI re-reads, so pushing it on every
        # token is pure load: a 500-token answer would mean 500 API writes plus 500
        # UI updates for a screen that cannot show more than a few per second.
        # ~8 writes per second keep the text visibly growing at a fraction of the cost.
        persist_interval_s = 0.12
        last_persist_s = 0.0
        last_persisted_text = ""

        for token in tokens:
            # TTFT fast-path: emit the first generated token immediately, before
            # waiting for a sentence terminator / next-token publish cycle.
            if not first_chunk_emitted and token:
                immediate = token.lstrip() if len(curr_text) == 0 else token
                if immediate:
                    feedback = _chat().Chat.Feedback()
                    feedback.text = immediate
                    feedback.text_type = _chat().Chat.Goal.TEXT_TYPE_SENTENCE
                    goal_handle.publish_feedback(feedback)
                    first_chunk_emitted = True
                    published_speech.append(immediate)
                    elapsed_ms = (_chat().time.monotonic() - t0) * 1000.0
                    self._remember_first_token_latency(chat_id, elapsed_ms)
                    self._record_local_turn_stage("first_token", t0)
                    self.get_logger().info(
                        f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                        f"elapsed_ms={elapsed_ms:.2f}"
                    )

            # Blockly code blocks still publish on the next token; speech does not.
            if (
                prev_text is not None
                and prev_text_type == _chat().Chat.Goal.TEXT_TYPE_CODE_VISUAL
            ):
                feedback = _chat().Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

            # Accumulate token (strip leading spaces if first)
            piece = token if len(curr_text) > 0 else token.lstrip()
            curr_text = curr_text + piece
            raw_answer = raw_answer + piece

            # Strip off complete chunks (code/sentences)
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return prev_text, prev_text_type, curr_text

                # Visual code block
                code_visual_match = code_visual_pattern.search(curr_text)
                if code_visual_match is not None:
                    code_visual = code_visual_match.group(1)
                    prev_text = code_visual
                    prev_text_type = _chat().Chat.Goal.TEXT_TYPE_CODE_VISUAL
                    curr_text = curr_text[code_visual_match.end() :].rstrip()
                    continue

                # Sentence
                sentence_match = sentence_pattern.search(curr_text)
                if sentence_match is not None:
                    sentence = sentence_match.group(1) + (
                        sentence_match.group(3)
                        if sentence_match.group(3) is not None
                        else ""
                    )
                    prev_text = sentence
                    prev_text_type = _chat().Chat.Goal.TEXT_TYPE_SENTENCE
                    if not first_sentence_flushed:
                        first_sentence_flushed = True
                        elapsed_ms = (_chat().time.monotonic() - t0) * 1000.0
                        self._record_local_turn_stage("first_sentence_flushed", t0)
                        self.get_logger().info(
                            f"[PERF_TRACE] FIRST_SENTENCE_FLUSHED chat={chat_id} "
                            f"elapsed_ms={elapsed_ms:.2f}"
                        )
                    curr_text = curr_text[
                        sentence_match.end(
                            3 if sentence_match.group(3) is not None else 1
                        ) :
                    ].rstrip()
                    continue

                break

            if self._publish_ready_clauses(
                goal_handle,
                raw_answer,
                published_speech,
                chat_id=chat_id,
                t0=t0 if not first_sentence_flushed else None,
            ):
                first_sentence_flushed = True

            if raw_answer and raw_answer != last_persisted_text:
                now_s = _chat().time.monotonic()
                if now_s - last_persist_s >= persist_interval_s:
                    self._persist_streaming_assistant_text(
                        chat_id,
                        raw_answer,
                        bool_update_chat_message,
                    )
                    bool_update_chat_message = True
                    last_persisted_text = raw_answer
                    last_persist_s = now_s

        # The throttle may have swallowed the final chunk (and a reply can end
        # without a terminator at all), so the last state is always written once.
        if raw_answer and raw_answer != last_persisted_text:
            self._persist_streaming_assistant_text(
                chat_id,
                raw_answer,
                bool_update_chat_message,
            )
            last_persisted_text = raw_answer

        # A reply can end without a sentence terminator. That tail is part of
        # the answer, so it is persisted here instead of being dropped.
        leftover = curr_text.strip()
        if leftover:
            if not first_chunk_emitted:
                first_chunk_emitted = True
                elapsed_ms = (_chat().time.monotonic() - t0) * 1000.0
                self._remember_first_token_latency(chat_id, elapsed_ms)
                self._record_local_turn_stage("first_token", t0)
                self.get_logger().info(
                    f"[PERF_TRACE] FIRST_CHUNK_EMITTED chat={chat_id} "
                    f"elapsed_ms={elapsed_ms:.2f}"
                )
            if self._publish_ready_clauses(
                goal_handle,
                raw_answer,
                published_speech,
                chat_id=chat_id,
                t0=t0 if not first_sentence_flushed else None,
            ):
                first_sentence_flushed = True
            if (
                prev_text is not None
                and prev_text_type == _chat().Chat.Goal.TEXT_TYPE_CODE_VISUAL
            ):
                feedback = _chat().Chat.Feedback()
                feedback.text = prev_text
                feedback.text_type = prev_text_type
                goal_handle.publish_feedback(feedback)
                prev_text = None
                prev_text_type = None

        if prev_text_type == _chat().Chat.Goal.TEXT_TYPE_CODE_VISUAL:
            return prev_text, prev_text_type, curr_text

        speech_tail = speech_result_tail(raw_answer, published_speech)
        return None, None, speech_tail

    def _remember_first_token_latency(self, chat_id: str, elapsed_ms: float) -> None:
        """Persist the measurement. A down API must not fail the turn."""
        try:
            _chat().voice_assistant_client.record_first_token_latency(
                chat_id, elapsed_ms
            )
        except Exception as exc:
            self.get_logger().warning(
                f"first-token latency was not stored for chat {chat_id}: {exc}"
            )

    def _publish_ready_clauses(
        self,
        goal_handle,
        raw_answer: str,
        published_speech: list[str],
        chat_id: str = "",
        t0: Optional[float] = None,
    ) -> bool:
        """Publish the next incremental speech delta so TTS can start early."""
        flushed_any = False
        while True:
            delta = next_speech_delta(raw_answer, published_speech)
            if not delta:
                break
            feedback = _chat().Chat.Feedback()
            feedback.text = delta
            feedback.text_type = _chat().Chat.Goal.TEXT_TYPE_SENTENCE
            goal_handle.publish_feedback(feedback)
            if not flushed_any and t0 is not None:
                elapsed_ms = (_chat().time.monotonic() - t0) * 1000.0
                self._record_local_turn_stage("first_sentence_flushed", t0)
                self.get_logger().info(
                    f"[PERF_TRACE] FIRST_SENTENCE_FLUSHED chat={chat_id} "
                    f"elapsed_ms={elapsed_ms:.2f}"
                )
            flushed_any = True
            published_speech.append(delta)
        return flushed_any

    def _hermes_timeout(self) -> int:
        """Timeout for one hermes turn, read live so ops can tune it via env."""
        return int(
            os.environ.get(
                "PIB_HERMES_TIMEOUT",
                _chat().hermes_agent_client.DEFAULT_TIMEOUT_SECONDS,
            )
        )

    def _run_hermes_turn(
        self,
        text: str,
        chat_id: str,
        personality_id: Optional[str],
        description: str,
        timeout: Optional[int] = None,
        goal_handle=None,
        model: Optional[str] = None,
        provider: Optional[str] = None,
        base_url: Optional[str] = None,
        tool_calling: bool = True,
        message_history: Optional[int] = None,
    ) -> str:
        """Run one hermes turn and return speakable text. Never raises.

        Deliberately synchronous and asyncio-free: rclpy drives the action
        server's execute_callback with its own executor, so the calling thread
        has no asyncio event loop and asyncio.get_running_loop() would raise
        RuntimeError('no running event loop').

        Returns "" only when the goal was cancelled while the agent was running;
        every agent failure yields the fallback sentence so the goal can still
        succeed.
        """
        if timeout is None:
            timeout = self._hermes_timeout()

        def _turn() -> str:
            # Smart turns seed the Hermes session from flask chat messages (the
            # record the user saw) before the agent runs; persona still comes
            # from the profile (-p).
            # When the warm daemon is already up, skip filesystem profile
            # re-validation — profiles were provisioned at personality create
            # time / prior cold starts and re-writing SOUL.md adds multi-second
            # latency on the Pi.
            # Always ensure new profile directories receive config.yaml / .env
            if personality_id:
                pdir = _chat().hermes_agent_client.profile_dir_for(personality_id)
                cfg_file = os.path.join(pdir, "config.yaml")
                if (
                    not os.path.exists(cfg_file)
                    or not _chat().hermes_agent_client.is_warm_daemon_active()
                ):
                    _chat().hermes_agent_client.ensure_profile(
                        personality_id,
                        soul_text=description,
                        model=model,
                        provider=provider,
                        base_url=base_url,
                        tool_calling=tool_calling,
                    )
            disabled_toolsets = (
                _chat().hermes_agent_client.DEFAULT_DISABLED_TOOLSETS
                if tool_calling
                else _chat().hermes_agent_client.disabled_toolsets_for_turn(False)
            )
            enabled_toolsets = (
                _chat().hermes_agent_client.DEFAULT_ENABLED_TOOLSETS
                if tool_calling
                else _chat().hermes_agent_client.enabled_toolsets_for_turn(False)
            )

            def _on_subprocess_fallback(reason: str) -> None:
                self._emit_hermes_subprocess_fallback_notice(reason)

            return _chat().hermes_agent_client.run_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                toolsets=disabled_toolsets,
                enabled_toolsets=enabled_toolsets,
                timeout=timeout,
                message_history=message_history,
                on_subprocess_fallback=_on_subprocess_fallback,
            )

        future = self._hermes_executor.submit(_turn)
        deadline = (
            _chat().time.monotonic() + timeout + _chat().HERMES_WAIT_GRACE_SECONDS
        )

        while True:
            try:
                return future.result(timeout=_chat().HERMES_CANCEL_POLL_SECONDS)
            except FutureTimeoutError:
                pass
            except _chat().direct_tool_loop.DirectToolLoopError:
                # The store could not supply the provider key. The goal aborts
                # with that message; the fallback sentence would hide it.
                raise
            except Exception as exc:
                self.get_logger().error(f"hermes agent turn failed: {exc}")
                return _chat().hermes_agent_client.FALLBACK_REPLY

            if goal_handle is not None and goal_handle.is_cancel_requested:
                # The worker is left to finish on its own; the shared pool
                # outlives this goal.
                self.get_logger().info(
                    f"hermes turn abandoned, goal cancelled (chat={chat_id})"
                )
                return ""

            if _chat().time.monotonic() > deadline:
                self.get_logger().error(
                    f"hermes turn exceeded {timeout}s plus grace "
                    f"(chat={chat_id}); answering with the fallback reply"
                )
                return _chat().hermes_agent_client.FALLBACK_REPLY

    def _stream_hermes_turn(
        self,
        text: str,
        chat_id: str,
        personality_id: Optional[str],
        description: str,
        goal_handle=None,
        model: Optional[str] = None,
        provider: Optional[str] = None,
        base_url: Optional[str] = None,
        tool_calling: bool = True,
        message_history: Optional[int] = None,
    ):
        """Yield daemon deltas, retrying through the non-streaming path on error."""
        timeout = self._hermes_timeout()
        events: Queue = Queue()

        def _consume_stream() -> None:
            try:
                if personality_id:
                    pdir = _chat().hermes_agent_client.profile_dir_for(personality_id)
                    cfg_file = os.path.join(pdir, "config.yaml")
                    if (
                        not os.path.exists(cfg_file)
                        or not _chat().hermes_agent_client.is_warm_daemon_active()
                    ):
                        _chat().hermes_agent_client.ensure_profile(
                            personality_id,
                            soul_text=description,
                            model=model,
                            provider=provider,
                            base_url=base_url,
                            tool_calling=tool_calling,
                        )
                disabled_toolsets = (
                    _chat().hermes_agent_client.DEFAULT_DISABLED_TOOLSETS
                    if tool_calling
                    else _chat().hermes_agent_client.disabled_toolsets_for_turn(False)
                )
                enabled_toolsets = (
                    _chat().hermes_agent_client.DEFAULT_ENABLED_TOOLSETS
                    if tool_calling
                    else _chat().hermes_agent_client.enabled_toolsets_for_turn(False)
                )
                hermes_tools: list = []
                try:
                    for delta in _chat().hermes_agent_client.stream_turn(
                        text=text,
                        chat_id=chat_id,
                        personality_id=personality_id,
                        toolsets=disabled_toolsets,
                        enabled_toolsets=enabled_toolsets,
                        timeout=timeout,
                        tool_sink=hermes_tools,
                        message_history=message_history,
                    ):
                        events.put(("delta", delta))
                    for record in hermes_tools:
                        self._record_tool_call(
                            {
                                "name": str(record.get("name") or "tool"),
                                "ok": bool(record.get("ok", True)),
                                "argNames": [],
                            }
                        )
                    events.put(("done", None))
                except Exception as exc:
                    events.put(("error", exc))
            except Exception as exc:
                events.put(("error", exc))

        self._hermes_executor.submit(_consume_stream)
        deadline = (
            _chat().time.monotonic() + timeout + _chat().HERMES_WAIT_GRACE_SECONDS
        )
        emitted = ""

        while True:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return
            if _chat().time.monotonic() > deadline:
                event = ("error", TimeoutError("Hermes stream exceeded its deadline"))
            else:
                try:
                    event = events.get(timeout=_chat().HERMES_CANCEL_POLL_SECONDS)
                except Empty:
                    continue

            kind, value = event
            if kind == "delta":
                emitted += value
                yield value
                continue
            if kind == "done":
                return
            if isinstance(value, _chat().direct_tool_loop.DirectToolLoopError):
                raise value

            self.get_logger().warning(
                f"hermes streaming failed (chat={chat_id}): {value}; "
                "retrying without streaming"
            )
            reply = self._run_hermes_turn(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                description=description,
                timeout=timeout,
                goal_handle=goal_handle,
                model=model,
                provider=provider,
                base_url=base_url,
                tool_calling=tool_calling,
                message_history=message_history,
            )
            if emitted and reply.startswith(emitted):
                reply = reply[len(emitted) :]
            if reply:
                yield reply
            return

    def _robot_context(self) -> str:
        """The body description for this turn, empty when the API cannot be asked."""
        try:
            return _chat().robot_context.get_robot_context()
        except Exception:
            return ""

    def _iter_local_tokens(self, chat_id: str, content: str, system_prompt: str):
        """Tokens from the on-device model. No provider key is read."""
        history: list[tuple[str, bool]] = []
        try:
            with self.voice_assistant_client_lock:
                (
                    successful,
                    chat_messages,
                ) = _chat().voice_assistant_client.get_chat_history(
                    chat_id, self.history_length
                )
            if successful:
                history = [
                    (message.content, message.is_user) for message in chat_messages
                ]
        except Exception as exc:
            self.get_logger().error(
                f"local model history could not be read for chat {chat_id}: {exc}"
            )
        return iter_reply(system_prompt, history, content)

    def _get_turn_lock(self) -> Lock:
        """Return the per-node turn serialization lock (created lazily for test doubles)."""
        lock = getattr(self, "_turn_lock", None)
        if lock is None:
            lock = Lock()
            self._turn_lock = lock
        return lock

    def _stream_pib_cloud_turn(
        self,
        *,
        chat_id: str,
        content: str,
        personality: Any,
        plan: Any,
        generate_code: bool,
        tool_calling: Optional[bool] = None,
        goal_handle=None,
    ):
        """Stream SSE tokens from pib.Cloud (POST /v1/chat/completions, stream=true)."""
        history: list[tuple[str, bool]] = []
        try:
            with self.voice_assistant_client_lock:
                (
                    successful,
                    chat_messages,
                ) = _chat().voice_assistant_client.get_chat_history(
                    chat_id, self.history_length
                )
            if successful and chat_messages:
                history = [
                    (message.content, message.is_user) for message in chat_messages
                ]
        except Exception as exc:
            self.get_logger().warning(
                f"pib.Cloud history could not be read for chat {chat_id}: {exc}"
            )

        system_prompt = direct_system_prompt(
            getattr(personality, "description", None),
            images_shown=True,
            robot_context=self._robot_context(),
            code_generation=generate_code,
        )
        system_prompt = plan.for_direct(system_prompt)
        endpoint_base = getattr(
            getattr(personality, "assistant_model", None), "endpoint_base", None
        )
        if tool_calling is None:
            tool_calling = _chat().direct_tool_loop.tool_calling_enabled(personality)
        allow_image = _chat().direct_tool_loop.image_tool_allowed(
            personality, tool_calling
        )
        declarations = (
            _chat().direct_tool_loop.mcp_tool_declarations() if tool_calling else []
        )
        tools = _chat().direct_tool_loop.tools_for_turn(
            declarations,
            tool_calling=tool_calling,
            allow_image=allow_image,
        )
        return _chat().pib_cloud_stream.stream_pib_cloud_chat(
            system_prompt=system_prompt,
            user_text=content,
            history=history,
            chat_id=chat_id,
            endpoint_base=endpoint_base,
            tools=tools,
            allow_image=allow_image,
            on_tool_call=getattr(self, "_record_tool_finished", None),
            on_tool_start=getattr(self, "_record_tool_started", None),
            local_fallback=lambda: self._iter_local_tokens(
                chat_id, content, system_prompt
            ),
            is_cancelled=lambda: bool(
                goal_handle is not None and goal_handle.is_cancel_requested
            ),
        )

    def _execute_chat_turn(self, goal_handle: ServerGoalHandle):
        t0 = _chat().time.monotonic()
        self._ensure_turn_trace_anchor(t0)
        self.get_logger().info("start chat request")

        # Unpack request data
        request: _chat().Chat.Goal = goal_handle.request
        chat_id: str = request.chat_id
        content: str = request.text
        generate_code: bool = request.generate_code
        turn_id = getattr(request, "turn_id", "") or ""
        self._turn_chat_id = chat_id
        self._turn_trace_turn_id = turn_id
        if turn_id:
            self._emit_turn_state(turn_id, STATE_RUNNING, chat_id)

        self.get_logger().info(
            f"[PERF_TRACE] ROS_SERVICE_RECV chat={chat_id} elapsed_ms=0.00"
        )

        # Create the user message (first chunk) via Action path helper. This
        # write also sets last_pib_message_id, so it has to land before the
        # assistant chunks start creating and updating their own message.
        self.create_chat_message(chat_id, content, True, False, True)

        # Turn toward the detected speaker before any answer, including the
        # degraded-mode sentence. The stored user line stays the words they said.
        plan = self._attention_plan()

        # Get personality (also sets how much history to include)
        with self.voice_assistant_client_lock:
            successful, personality = (
                _chat().voice_assistant_client.get_personality_from_chat(chat_id)
            )
        if not successful or personality is None:
            personality_id = getattr(personality, "personality_id", None)
            if personality_id:
                self.get_logger().error(
                    f"chat could not be resolved chat={chat_id} "
                    f"personality={personality_id}"
                )
            else:
                self.get_logger().error(f"chat could not be resolved chat={chat_id}")
            goal_handle.abort()
            return _chat().Chat.Result()
        self.history_length = personality.message_history
        description = personality_system_prompt(
            soul_text=getattr(personality, "description", None),
            robot_context=self._robot_context(),
            code_generation=generate_code,
        )

        # Channel is a personality setting, not the provider's api name.
        channel = turn_channel(personality)
        is_smart = channel == CHANNEL_SMART
        stored_channel = getattr(personality, "channel", None)
        if (
            channel == CHANNEL_DIRECT
            and isinstance(stored_channel, str)
            and stored_channel == CHANNEL_SMART
        ):
            self.get_logger().info(
                f"smart chats disabled; routing chat={chat_id} as direct"
            )
        else:
            self.get_logger().info(f"chat channel={channel} chat={chat_id}")

        api_name = getattr(
            getattr(personality, "assistant_model", None), "api_name", None
        )
        local_model = api_name == _chat().LOCAL_MODEL_API_NAME
        pib_cloud_model = api_name == _chat().PIB_CLOUD_API_NAME
        model_provider_name = _chat()._model_provider_name(api_name)
        tool_calling = _chat().direct_tool_loop.tool_calling_enabled(personality)
        # Old Direct text selections stop here, before the key store. A locked
        # or missing key must not hide the retirement reason, and the stored
        # selection is not migrated.
        if legacy_direct_text_selected(
            channel=channel,
            api_name=api_name,
            voice_mode=getattr(personality, "voice_mode", None),
        ):
            return self._finish_retired_direct_text(goal_handle, chat_id, api_name)
        # Only a model whose provider needs a key is gated by the store. A
        # cloud model does; the on-device model does not, and still answers
        # while the store is locked.
        if (
            _chat()._model_needs_provider_key(api_name)
            and self._key_store_mode() != MODE_UNLOCKED
        ):
            # Smart and Direct both stop here. The sentence is the personality
            # speaking: the assistant plays it with this personality's gender
            # and language on local Supertone. The goal succeeds.
            if goal_handle.is_cancel_requested:
                self._finish_cancelled_turn(chat_id, goal_handle)
                return _chat().Chat.Result()
            sentence = refusal_sentence(channel)
            self.create_chat_message(chat_id, sentence, False, False, True)
            goal_handle.succeed()
            result = _chat().Chat.Result()
            result.text = sentence
            result.text_type = _chat().Chat.Goal.TEXT_TYPE_SENTENCE
            self.get_logger().info(
                f"chat refused in degraded mode channel={channel} chat={chat_id}"
            )
            return result

        direct_turn_meta: dict[str, Any] | None = None
        try:
            if is_smart:
                turn_path = "Hermes"
                # The channel decides the transport: Smart runs on Hermes, Direct
                # never does. Tool calling says whether a turn offers MCP tools
                # inside whichever runtime serves it - it is not a transport
                # switch, so it must not move a Direct personality onto Hermes.
                # Smart stays on Hermes even for the on-device model: the model
                # decides what Hermes runs, never the transport.
                if goal_handle.is_cancel_requested:
                    self._finish_cancelled_turn(chat_id, goal_handle)
                    return _chat().Chat.Result()
                tokens = self._stream_hermes_turn(
                    text=plan.for_hermes(content),
                    chat_id=chat_id,
                    personality_id=getattr(personality, "personality_id", None),
                    description=description,
                    model=api_name,
                    provider=model_provider_name,
                    base_url=_chat()._model_base_url(api_name),
                    tool_calling=tool_calling,
                    goal_handle=goal_handle,
                    message_history=self.history_length,
                )
            elif pib_cloud_model:
                turn_path = "pib.Cloud"
                if goal_handle.is_cancel_requested:
                    self._finish_cancelled_turn(chat_id, goal_handle)
                    return _chat().Chat.Result()
                self.get_logger().info(
                    f"pib.Cloud streaming chat={chat_id} model={_chat().PIB_CLOUD_API_NAME}"
                )
                tokens = self._stream_pib_cloud_turn(
                    chat_id=chat_id,
                    content=content,
                    personality=personality,
                    plan=plan,
                    generate_code=generate_code,
                    tool_calling=tool_calling,
                    goal_handle=goal_handle,
                )
            elif local_model:
                turn_path = "the on-device model"
                system_prompt = direct_system_prompt(
                    personality.description,
                    images_shown=True,
                    robot_context=self._robot_context(),
                    code_generation=generate_code,
                )
                system_prompt = plan.for_direct(system_prompt)
                self.get_logger().info(
                    f"local model chat={chat_id} model={_chat().LOCAL_MODEL_API_NAME}"
                )
                tokens = self._iter_local_tokens(chat_id, content, system_prompt)
            else:
                turn_path = "the direct tool loop"
                # Pull recent message history for context
                with self.voice_assistant_client_lock:
                    (
                        successful,
                        chat_messages,
                    ) = _chat().voice_assistant_client.get_chat_history(
                        chat_id, self.history_length
                    )
                if not successful:
                    self.get_logger().error(
                        f"chat with id'{chat_id}' does not exist..."
                    )
                    goal_handle.abort()
                    return _chat().Chat.Result()
                message_history = [
                    PublicApiChatMessage(message.content, message.is_user)
                    for message in chat_messages
                ]

                # Direct: the SOUL text is the system prompt. MEMORY.md is not
                # read; that file belongs to the Hermes profile on Smart turns.
                # A camera frame is never attached here. It arrives only when
                # the model calls capture_image, and that tool is absent while
                # tool calling is off.
                system_prompt = direct_system_prompt(
                    personality.description,
                    images_shown=True,
                    robot_context=self._robot_context(),
                    code_generation=generate_code,
                )
                system_prompt = plan.for_direct(system_prompt)
                allow_image = _chat().direct_tool_loop.image_tool_allowed(
                    personality, tool_calling
                )
                api_name = personality.assistant_model.api_name
                if not _chat().direct_tool_loop.supports_tool_endpoint(api_name):
                    raise _chat().direct_tool_loop.DirectToolLoopError(
                        "Direct tool calling has no non-beta endpoint for "
                        f"model {api_name!r}. The pinned model is "
                        f"{_chat().direct_tool_loop.PINNED_MODEL} "
                        f"({_chat().direct_tool_loop.PINNED_PROVIDER}), checked "
                        f"{_chat().direct_tool_loop.PINNED_CHECKED_ON}."
                    )
                self.get_logger().info(
                    f"direct tool loop model={_chat().direct_tool_loop.PINNED_MODEL} "
                    f"provider={_chat().direct_tool_loop.PINNED_PROVIDER} chat={chat_id}"
                )
                personality_name = getattr(personality, "name", "") or ""
                direct_turn_meta = {
                    "started": _chat().time.monotonic(),
                    "tool_rounds": 0,
                }
                self._turn_direct_tool_rounds = 0
                self.get_logger().info(
                    f"direct turn started personality={personality_name} model={api_name}"
                )

                def count_direct_tool_round() -> None:
                    direct_turn_meta["tool_rounds"] += 1
                    self._turn_direct_tool_rounds = direct_turn_meta["tool_rounds"]

                def report_key_source(source: str) -> None:
                    # source is key-store or environment. The key itself
                    # is not passed here, so it cannot land in the log.
                    self.get_logger().info(
                        f"direct provider key source={source} "
                        f"provider={_chat().direct_tool_loop.PINNED_PROVIDER} "
                        f"chat={chat_id}"
                    )

                tokens = _chat().direct_tool_loop.run_direct_turn(
                    system_prompt=system_prompt,
                    user_text=content,
                    history=[
                        (message.content, message.is_user)
                        for message in message_history
                    ],
                    tool_calling=tool_calling,
                    allow_image=allow_image,
                    report_key_source=report_key_source,
                    on_tool_call=self._record_tool_finished,
                    on_tool_start=self._record_tool_started,
                    on_tool_image=self._record_tool_image,
                    on_tool_round=count_direct_tool_round,
                    session=self._direct_session(personality),
                    # Identity edits (description/SOUL) must replace the frozen prompt.
                    session_identity=self._direct_identity_fingerprint(personality),
                )

            # Which runtime serves this turn, before the first token is asked
            # for: a UI needs it even when the transport fails afterwards.
            self._emit_turn_path(chat_id=chat_id, path=turn_path, model=api_name)
            prev_text, prev_text_type, curr_text = self._stream_chunks_to_goal(
                goal_handle, chat_id, tokens, t0=t0
            )
            if direct_turn_meta is not None:
                try:
                    elapsed = _chat().time.monotonic() - direct_turn_meta["started"]
                    self.get_logger().info(
                        f"direct turn finished in {elapsed:.1f} s, "
                        f"{direct_turn_meta['tool_rounds']} tool rounds"
                    )
                except Exception:
                    pass
            if goal_handle.is_cancel_requested:
                self._finish_cancelled_turn(chat_id, goal_handle)
                return _chat().Chat.Result()
            self._finalize_message_meta(chat_id)
            if turn_id:
                self._emit_turn_state(turn_id, STATE_COMPLETED, chat_id)

        except Exception as e:
            # Name the path the turn actually took: a pib.Cloud turn with tools
            # enabled runs on Hermes, and a log that still says "direct" sends
            # the next diagnosis to the wrong component.
            # A 401 from a route the operator did not expect is unreadable without
            # the host: on .172 the pib.Cloud key looked broken while the profile
            # pointed at a LAN dev proxy (PR-2096).
            self.get_logger().error(
                f"failed to send request to {turn_path} "
                f"(chat={chat_id} model={api_name} provider_base_url={self._provider_base_url(api_name)}): {e}"
            )
            goal_handle.abort()
            return _chat().Chat.Result()

        # Finish Action: return the last pending chunk (if any)
        goal_handle.succeed()
        result = _chat().Chat.Result()
        if prev_text is None:
            result.text = curr_text
            result.text_type = _chat().Chat.Goal.TEXT_TYPE_SENTENCE
        else:
            result.text = prev_text
            result.text_type = prev_text_type
        self.get_logger().info(
            f"[PERF_TRACE] ROS_SERVICE_DONE chat={chat_id} "
            f"elapsed_ms={(_chat().time.monotonic() - t0) * 1000.0:.2f}"
        )
        return result
