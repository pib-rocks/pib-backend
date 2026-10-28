"""Turn trace and turn metadata helpers for ChatNode (PR-2120)."""

from __future__ import annotations

import json
import os
from typing import Any, Optional

from std_msgs.msg import String

from voice_assistant.chat_turn import STATE_CANCELLED, turn_state_event


def _chat():
    import voice_assistant.chat as mod

    return mod


class ChatTurnTraceMixin:
    TURN_PATH_TOKENS = {
        "Hermes": "hermes",
        "pib.Cloud": "pib-cloud",
        "the on-device model": "local",
        "the direct tool loop": "direct-tool-loop",
    }

    def _install_turn_trace_emitter(self) -> None:
        """Publish pib.Cloud stage events. A failure here must not stop the node."""
        try:
            from voice_assistant.turn_latency import TURN_TRACE_TOPIC

            publisher = self.create_publisher(String, TURN_TRACE_TOPIC, 10)
            self._turn_trace_publisher = publisher

            def emit(raw: str) -> None:
                # The same events a UI consumes also describe this turn in the
                # stored answer (PC-82), so record them before publishing.
                self._record_turn_trace(raw)
                message = String()
                message.data = raw
                publisher.publish(message)

            _chat().pib_cloud_stream.set_trace_emitter(emit)
        except Exception as exc:
            self.get_logger().warning(f"turn trace emitter was not installed: {exc}")

    def _turn_trace_chat_id(self) -> str:
        chat_id = getattr(self, "_turn_chat_id", None)
        return chat_id if isinstance(chat_id, str) else ""

    def _record_tool_started(self, record: dict) -> None:
        """Publish that a tool invocation has begun (no argument values)."""
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event, now_ms

            name = str(record.get("name") or "tool")
            call_id = record.get("callId") or record.get("call_id") or ""
            message = String()
            message.data = format_event(
                "tool_call_started",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail={
                    "call_id": call_id,
                    "arg_names": record.get("argNames") or [],
                },
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"tool start trace was not published: {exc}")

    def _record_tool_finished(self, record: dict) -> None:
        """Keep one executed tool of this turn and publish finished trace events.

        The record carries the tool name, its outcome, its duration and the names
        of its arguments - never the values (PC-84).
        """
        calls = getattr(self, "_turn_tool_calls", None)
        if not isinstance(calls, list):
            calls = self._turn_tool_calls = []
        calls.append(record)
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event, now_ms

            name = str(record.get("name") or "tool")
            call_id = record.get("callId") or record.get("call_id") or ""
            detail = {
                "call_id": call_id,
                "ok": bool(record.get("ok", True)),
                "duration_ms": record.get("durationMs"),
                "arg_names": record.get("argNames") or [],
                "error": record.get("error"),
            }
            message = String()
            message.data = format_event(
                "tool_call_finished",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail=detail,
            )
            publisher.publish(message)
            legacy = String()
            legacy.data = format_event(
                "tool_call",
                ts_ms=now_ms(),
                source=name,
                chat_id=self._turn_trace_chat_id(),
                detail=detail,
            )
            publisher.publish(legacy)
        except Exception as exc:
            self.get_logger().warning(f"tool call trace was not published: {exc}")

    def _record_tool_call(self, record: dict) -> None:
        """Hermes and other paths that only report after execution."""
        self._record_tool_finished(record)

    def _record_tool_image(self, record: dict) -> None:
        """Keep one picture a tool captured this turn, scaled down.

        Only the camera tool produces one, and only the direct tool loop reports
        it: the Hermes daemon runs its tools itself, so an image captured there
        never reaches this runtime.
        """
        prepared = _chat().prepare_tool_image(
            str(record.get("imageBase64") or ""),
            str(record.get("mimeType") or "image/jpeg"),
        )
        if prepared is None:
            return
        images = getattr(self, "_turn_images", None)
        if not isinstance(images, list):
            images = self._turn_images = []
        tool = record.get("tool")
        prepared["tool"] = tool if isinstance(tool, str) else ""
        images.append(prepared)

    def _reset_turn_meta(self) -> None:
        """A new turn forgets the previous turn's facts."""
        self._turn_stages = {}
        self._turn_started_ms = None
        self._turn_fallback_reason = None
        self._turn_stream_end = None
        self._turn_path_token = ""
        self._turn_model = ""
        self._turn_tool_calls = []
        self._turn_images = []
        self._turn_image_references = []
        self._turn_chat_id = ""
        self._turn_assistant_message_id = ""
        self._turn_trace_turn_id = ""
        self._turn_trace_cancelled = False
        self._turn_direct_tool_rounds = None

    def _ensure_turn_trace_anchor(self, t0: float) -> None:
        """Typed turns anchor at service recv when capture never sent end_of_speech."""
        if getattr(self, "_turn_started_ms", None) is None:
            self._turn_started_ms = float(t0) * 1000.0
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        if "ros_service_recv" not in stages:
            stages["ros_service_recv"] = 0.0

    def _record_local_turn_stage(self, stage: str, t0: float) -> None:
        """Record a stage the chat node observes locally (PR-2072)."""
        self._ensure_turn_trace_anchor(t0)
        started = getattr(self, "_turn_started_ms", None)
        if started is None:
            return
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        if stage in stages:
            return
        stages[stage] = round(_chat().time.monotonic() * 1000.0 - started, 1)

    def _freeze_turn_done_stage(self) -> None:
        if "turn_end" in getattr(self, "_turn_stages", {}) or "stream_done" in getattr(
            self, "_turn_stages", {}
        ):
            return
        started = getattr(self, "_turn_started_ms", None)
        if started is None:
            return
        stages = getattr(self, "_turn_stages", None)
        if not isinstance(stages, dict):
            stages = self._turn_stages = {}
        stages["turn_end"] = round(_chat().time.monotonic() * 1000.0 - started, 1)

    def _emit_turn_state(
        self, turn_id: str, state: str, chat_id: str, *, reason: str = ""
    ) -> None:
        """Publish running / cancelled / completed for one explicit turn id."""
        if not turn_id:
            return
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            message = String()
            message.data = turn_state_event(
                turn_id, state, chat_id=chat_id, reason=reason
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"turn state trace was not published: {exc}")

    def _discard_assistant_reply(self, chat_id: str) -> None:
        """Drop a partial assistant message after cancel (PR-2069)."""
        message_id = getattr(self, "_turn_assistant_message_id", None)
        if not message_id:
            return
        self._turn_assistant_message_id = ""
        try:
            with self.voice_assistant_client_lock:
                _chat().voice_assistant_client.delete_chat_message(chat_id, message_id)
        except Exception as exc:
            self.get_logger().warning(
                f"cancelled assistant message was not deleted chat={chat_id}: {exc}"
            )
        if getattr(self, "last_pib_message_id", None) == message_id:
            self.last_pib_message_id = None
            self.message_content = None

    def _finish_cancelled_turn(self, chat_id: str, goal_handle) -> None:
        """Stop the action, remove partial assistant text, and trace cancelled."""
        self._turn_trace_cancelled = True
        self._freeze_turn_done_stage()
        self._discard_assistant_reply(chat_id)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        turn_id = getattr(self, "_turn_trace_turn_id", "") or ""
        if turn_id:
            self._emit_turn_state(turn_id, STATE_CANCELLED, chat_id)

    def _record_turn_trace(self, raw: str) -> None:
        """Keep stage timings and the fallback reason of the running turn."""
        try:
            event = json.loads(raw)
        except (TypeError, ValueError):
            return
        stage = event.get("stage")
        ts = event.get("ts_ms")
        if stage == "end_of_speech":
            self._turn_stages = {}
            self._turn_started_ms = float(ts) if isinstance(ts, (int, float)) else None
        elif stage == "stream_done":
            self._turn_stream_end = "done"
        reason = (event.get("detail") or {}).get("fallback_reason")
        if isinstance(reason, str) and reason:
            self._turn_fallback_reason = reason
        started = getattr(self, "_turn_started_ms", None)
        if started is not None and isinstance(ts, (int, float)):
            stages = getattr(self, "_turn_stages", None)
            if not isinstance(stages, dict):
                stages = self._turn_stages = {}
            stages[stage] = round(float(ts) - started, 1)

    def _emit_hermes_subprocess_fallback_notice(self, reason: str) -> None:
        """Tell the voice layer to speak before the oneshot subprocess runs."""
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event, now_ms

            raw = format_event(
                "hermes_subprocess_fallback",
                ts_ms=now_ms(),
                chat_id=self._turn_trace_chat_id(),
                detail={"fallback_reason": reason},
            )
            self._record_turn_trace(raw)
            message = String()
            message.data = raw
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(
                f"hermes subprocess fallback notice was not published: {exc}"
            )

    def _turn_meta(self) -> dict | None:
        """Facts of this turn, stored with the answer (PC-82).

        Empty for a turn without a transport announcement - a user message or a
        turn that never reached a transport must not invent metadata.
        """
        meta: dict = {}
        source = getattr(self, "_turn_path_token", "")
        model = getattr(self, "_turn_model", "")
        stages = getattr(self, "_turn_stages", None)
        reason = getattr(self, "_turn_fallback_reason", None)
        stream_end = getattr(self, "_turn_stream_end", None)
        if source:
            meta["source"] = source
        if model:
            meta["model"] = model
        if isinstance(stages, dict) and stages:
            meta["stagesMs"] = dict(stages)
        if reason:
            meta["fallbackReason"] = reason
        if stream_end:
            meta["streamEnd"] = stream_end
        tool_calls = getattr(self, "_turn_tool_calls", None)
        if isinstance(tool_calls, list) and tool_calls:
            meta["toolCalls"] = list(tool_calls)
        images = getattr(self, "_turn_image_references", None)
        if isinstance(images, list) and images:
            meta["images"] = list(images)
        if isinstance(stages, dict) and stages:
            from voice_assistant.turn_latency import message_turn_trace

            trace = message_turn_trace(
                stages,
                cancelled=bool(getattr(self, "_turn_trace_cancelled", False)),
                tool_rounds=getattr(self, "_turn_direct_tool_rounds", None),
                tool_calls=tool_calls if isinstance(tool_calls, list) else None,
            )
            if trace:
                meta["turnTrace"] = trace
        return meta or None

    @staticmethod
    def _provider_base_url(api_name: str) -> str:
        """Host a chat turn talks to, so a provider error can be located.

        Falls back to the documented cloud default when nothing is configured:
        the operator needs the host more than a pretty log line.
        """
        route = (os.getenv("PIB_CLOUD_BASE_URL") or "").strip().rstrip("/")
        return route or "default"

    def _finalize_message_meta(self, chat_id: str) -> None:
        """Write the completed turn facts onto the answer that was just stored.

        Pictures a tool captured are uploaded here rather than before the answer
        exists: an image is attached to a message id, and the id only exists once
        the answer was stored. What the message carries is the reference the API
        answers, never the bytes (PC-119).
        """
        try:
            self._freeze_turn_done_stage()
            self._attach_turn_images(chat_id)
            meta = self._turn_meta()
            message_id = getattr(self, "last_pib_message_id", None)
            if not meta or not message_id:
                return
            with self.voice_assistant_client_lock:
                successful, _ = _chat().voice_assistant_client.update_chat_message(
                    chat_id,
                    getattr(self, "message_content", ""),
                    False,
                    message_id,
                    meta=meta,
                )
            if not successful:
                self.get_logger().warning(
                    f"turn metadata was not stored chat={chat_id}"
                )
        except Exception as exc:
            self.get_logger().warning(f"turn metadata failed: {exc}")
        finally:
            # The next turn starts clean; a recorder end_of_speech sets its anchor.
            self._reset_turn_meta()

    def _attach_turn_images(self, chat_id: str) -> None:
        """Upload the pictures of this turn and add their references to the meta."""
        images = getattr(self, "_turn_images", None)
        message_id = getattr(self, "last_pib_message_id", None)
        if not isinstance(images, list) or not images or not message_id:
            return
        references = []
        for image in images:
            try:
                with self.voice_assistant_client_lock:
                    reference = _chat().voice_assistant_client.add_chat_message_image(
                        chat_id, message_id, image["imageBase64"]
                    )
            except (
                Exception
            ) as exc:  # noqa: BLE001 - the answer stands without a picture
                self.get_logger().warning(f"captured image was not stored: {exc}")
                continue
            if not reference:
                self.get_logger().warning(
                    f"captured image was refused chat={chat_id} tool={image.get('tool')}"
                )
                continue
            references.append(reference)
        if references:
            self._turn_image_references = references

    #: Machine token per transport for the turn-path trace event. A UI reads this
    #: to say which runtime answered; the human words above it stay for the log.
    TURN_PATH_TOKENS = {
        "Hermes": "hermes",
        "pib.Cloud": "pib-cloud",
        "the on-device model": "local",
        "the direct tool loop": "direct-tool-loop",
    }

    def _emit_turn_path(self, *, chat_id: str, path: str, model: str | None) -> None:
        """Announce the transport and model of this turn on the trace topic.

        The trace already carries per-stage timings from the recorder, the player
        and the cloud client. None of them names the model that answered, so a UI
        could not tell pib.Cloud from the on-device fallback. This event closes
        that gap; a failure here must not stop the turn.
        """
        publisher = getattr(self, "_turn_trace_publisher", None)
        if publisher is None:
            return
        try:
            from voice_assistant.turn_latency import format_event

            self._turn_path_token = self.TURN_PATH_TOKENS.get(path, path)
            self._turn_model = model or ""
            detail = {"model": model} if model else None
            message = String()
            message.data = format_event(
                "turn_path",
                source=self.TURN_PATH_TOKENS.get(path, path),
                chat_id=chat_id,
                detail=detail,
            )
            publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"turn path trace was not published: {exc}")
