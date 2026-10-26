#!/usr/bin/env python3
"""Fixed-utterance latency harness. Run this inside the voice container.

It drives the real voice-assistant services and waits until each typed turn
shows up as a finished record in the JSONL the assistant appends. It does not
open a microphone and it does not print credentials.

Services (names and fields taken from the voice assistant node):

* ``get_voice_assistant_state`` / ``set_voice_assistant_state``
  (``GetVoiceAssistantState`` / ``SetVoiceAssistantState``; the state message
  carries ``turned_on`` and ``chat_id``)
* ``get_chat_is_listening`` (``GetChatIsListening``: ``chat_id`` -> ``listening``)
* ``send_chat_message`` (``SendChatMessage``: ``chat_id``, ``content`` -> ``successful``)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_VOICE = _REPO / "ros_packages" / "voice_assistant"
if str(_VOICE) not in sys.path:
    sys.path.insert(0, str(_VOICE))

# Same five sentences on every run. Lengths differ so p50/p95 are not one point.
FIXED_UTTERANCES = (
    "Hallo.",
    "Wie geht es dir heute?",
    "Bitte erzähl mir, was du heute schon erlebt hast.",
    "Kannst du in ein paar Sätzen erklären, warum der Himmel am Tag blau und am Abend rot erscheint?",
    "Ich hätte gern eine kleine Geschichte über einen neugierigen Roboter, der zum ersten Mal Schnee sieht, ihn anfasst und sich fragt, warum er in der warmen Hand verschwindet.",
)

DEFAULT_LOG = "/data/voice/turn-latency/turns.jsonl"


def _utterance(index: int) -> str:
    override = os.environ.get("MEASURE_TURN_LATENCY_UTTERANCE")
    if isinstance(override, str) and override.strip():
        return override.strip()
    return FIXED_UTTERANCES[index % len(FIXED_UTTERANCES)]


def _call(node, client, request, timeout_sec: float):
    import rclpy

    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("ROS service did not appear in time")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise TimeoutError("ROS service call did not finish")
    return future.result()


def _copy_state(target, source) -> None:
    target.turned_on = bool(source.turned_on)
    target.chat_id = source.chat_id or ""
    target.live_session = bool(getattr(source, "live_session", False))
    target.personality_id = getattr(source, "personality_id", "") or ""
    target.speaking = bool(getattr(source, "speaking", False))
    target.using_fallback = bool(getattr(source, "using_fallback", False))
    target.personality_name = getattr(source, "personality_name", "") or ""


def _read_new_records(path: Path, offset: int) -> tuple[list[dict], int]:
    if not path.is_file():
        return [], offset
    with path.open("r", encoding="utf-8") as handle:
        handle.seek(offset)
        chunk = handle.read()
        new_offset = handle.tell()
    records = []
    for line in chunk.splitlines():
        text = line.strip()
        if not text:
            continue
        try:
            item = json.loads(text)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            records.append(item)
    return records, new_offset


def _wait_for_turn(path: Path, offset: int, chat_id: str, timeout_sec: float):
    from voice_assistant.turn_latency import render_line

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        records, offset = _read_new_records(path, offset)
        for record in records:
            stages = (
                record.get("stages") if isinstance(record.get("stages"), dict) else {}
            )
            same_chat = not chat_id or record.get("chat_id") in ("", chat_id)
            if same_chat and "turn_end" in stages:
                print(render_line(record))
                return offset
        time.sleep(0.2)
    print("TIMEOUT waiting for turn")
    return None


def _ensure_listening(node, clients, chat_id: str, timeout_sec: float) -> None:
    from datatypes.srv import GetChatIsListening

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        request = GetChatIsListening.Request()
        request.chat_id = chat_id
        response = _call(node, clients["listening"], request, timeout_sec=5.0)
        if bool(getattr(response, "listening", False)):
            return
        time.sleep(0.2)
    raise TimeoutError("voice assistant did not start listening")


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chat-id", required=True)
    parser.add_argument("--turns", type=int, default=5)
    parser.add_argument("--log", default=DEFAULT_LOG)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args(argv)
    if args.turns < 1:
        print("measure_turn_latency: --turns must be at least 1", file=sys.stderr)
        return 2

    import rclpy
    from datatypes.msg import VoiceAssistantState
    from datatypes.srv import (
        GetVoiceAssistantState,
        SendChatMessage,
        SetVoiceAssistantState,
        GetChatIsListening,
    )

    rclpy.init()
    node = rclpy.create_node("turn_latency_measure")
    clients = {
        "get": node.create_client(GetVoiceAssistantState, "get_voice_assistant_state"),
        "set": node.create_client(SetVoiceAssistantState, "set_voice_assistant_state"),
        "send": node.create_client(SendChatMessage, "send_chat_message"),
        "listening": node.create_client(GetChatIsListening, "get_chat_is_listening"),
    }
    original = None
    changed = False
    exit_code = 0
    try:
        original_response = _call(
            node, clients["get"], GetVoiceAssistantState.Request(), args.timeout
        )
        original = original_response.voice_assistant_state
        already_on = (
            bool(original.turned_on) and (original.chat_id or "") == args.chat_id
        )
        if not already_on:
            state = VoiceAssistantState()
            state.turned_on = True
            state.chat_id = args.chat_id
            request = SetVoiceAssistantState.Request()
            request.voice_assistant_state = state
            response = _call(node, clients["set"], request, args.timeout)
            changed = True
            if not bool(getattr(response, "successful", False)):
                # Already-on for this chat is reported as unsuccessful by update_state.
                check = _call(
                    node,
                    clients["get"],
                    GetVoiceAssistantState.Request(),
                    args.timeout,
                )
                current = check.voice_assistant_state
                if not (
                    bool(current.turned_on) and (current.chat_id or "") == args.chat_id
                ):
                    print(
                        "measure_turn_latency: set_voice_assistant_state failed",
                        file=sys.stderr,
                    )
                    return 1
        _ensure_listening(node, clients, args.chat_id, args.timeout)
        log_path = Path(args.log)
        offset = log_path.stat().st_size if log_path.is_file() else 0
        for index in range(args.turns):
            # The turn is over in the log before the assistant has finished its
            # start signal and started listening again; a message sent into that
            # window is refused, so wait for the listening state first.
            _ensure_listening(node, clients, args.chat_id, args.timeout)
            request = SendChatMessage.Request()
            request.chat_id = args.chat_id
            request.content = _utterance(index)
            response = None
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                response = _call(node, clients["send"], request, args.timeout)
                if bool(getattr(response, "successful", False)):
                    break
                time.sleep(0.5)
            if response is None or not bool(getattr(response, "successful", False)):
                print(
                    "measure_turn_latency: send_chat_message was not accepted",
                    file=sys.stderr,
                )
                return 1
            offset = _wait_for_turn(log_path, offset, args.chat_id, args.timeout)
            if offset is None:
                return 1
        return 0
    except Exception as exc:
        print(f"measure_turn_latency: {exc}", file=sys.stderr)
        exit_code = 1
        return exit_code
    finally:
        if changed and original is not None:
            try:
                from datatypes.msg import VoiceAssistantState
                from datatypes.srv import SetVoiceAssistantState

                state = VoiceAssistantState()
                _copy_state(state, original)
                request = SetVoiceAssistantState.Request()
                request.voice_assistant_state = state
                _call(node, clients["set"], request, args.timeout)
            except Exception as exc:
                print(
                    f"measure_turn_latency: could not restore voice state: {exc}",
                    file=sys.stderr,
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(run())
