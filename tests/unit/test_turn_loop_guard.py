"""The turn loop must not run by itself (PC-40).

Measured on the test robot (chat 21ee3862, personality jb2, 2026-10-14
23:26-23:34): seven turns in four minutes, **every one of them with an empty
transcript** - nobody spoke - and each of them was sent to the chat anyway,
answered with a filler sentence and played back. Answer audio per empty turn:
6.5 s at the median, up to 54.7 s. The operator heard the robot answer again
and again, with long gaps, in an order that had nothing to do with anything he
said.

These tests pin the two guards:

* An empty transcript does not open a chat turn (the loop's engine).
* A recording window does not open while the previous answer's audio is still
  queued or playing (so two turns can never share the player's queue).
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from pib_hermes_config.turn_taking import transcript_opens_turn  # noqa: E402

ASSISTANT = VOICE_ROOT / "voice_assistant" / "assistant.py"


def test_only_spoken_text_opens_a_turn():
    assert transcript_opens_turn("Wie viele Arme hast du?") is True
    assert transcript_opens_turn("  ok  ") is True
    # Everything a silent window can produce:
    assert transcript_opens_turn("") is False
    assert transcript_opens_turn("   ") is False
    assert transcript_opens_turn("\n\t ") is False
    assert transcript_opens_turn(None) is False
    # Not a string at all must not be read as speech either.
    assert transcript_opens_turn(0) is False
    assert transcript_opens_turn([]) is False


def test_the_assistant_asks_that_question_before_it_chats():
    source = ASSISTANT.read_text(encoding="utf-8")
    assert "transcript_opens_turn(" in source
    # The guard sits in on_transcribed_text_received, before _start_spoken_turn
    # and before the chat call.
    received = source.index("def on_transcribed_text_received")
    block = source[received : received + 1600]
    assert "transcript_opens_turn(" in block
    assert block.index("transcript_opens_turn(") < block.index("self.chat(")
    # And it must not start a spoken turn for an empty window.
    assert block.index("transcript_opens_turn(") < block.index(
        "self._start_spoken_turn()"
    )


def test_an_empty_window_only_restarts_the_listening_cycle():
    """No chat, no speech: the start signal re-opens the recording window."""
    source = ASSISTANT.read_text(encoding="utf-8")
    received = source.index("def on_transcribed_text_received")
    block = source[received : received + 1600]
    assert "nothing transcribed in this window" in block
    assert "START_SIGNAL_FILE" in block
    # The empty branch returns before anything is played to the user.
    empty_branch = block[block.index("nothing transcribed in this window") :]
    assert "play_audio_from_speech" not in empty_branch.split("def ", 1)[0]


def test_a_recording_window_waits_for_the_answer_audio_to_drain():
    source = ASSISTANT.read_text(encoding="utf-8")
    start = source.index("def on_start_signal_played")
    block = source[start : start + 1200]
    assert "_answer_audio_pending" in block
    # The guard must come before the recording is opened.
    assert block.index("_answer_audio_pending") < block.index("self.record_audio(")
    # Waiting is a re-queued start signal, never a silent drop.
    assert "play_audio_from_file(" in block


def test_the_pending_flag_is_set_and_cleared_on_both_ends():
    source = ASSISTANT.read_text(encoding="utf-8")
    # Set when a clause is handed to the player.
    queued = source[source.index("def on_sentence_received") :]
    queued = queued[: queued.index("def on_code_visual_received")]
    assert "_answer_audio_pending = True" in queued
    # Cleared when the player drained it and at the start of a new turn.
    drained = source[source.index("def on_final_sentence_played") :]
    assert "_answer_audio_pending = False" in drained[:400]
    fresh = source[source.index("def _start_spoken_turn") :]
    assert "_answer_audio_pending = False" in fresh[:400]
    # The flag exists as state, not only inside the handler.
    assert (
        "self._answer_audio_pending = False"
        in source[: source.index("def on_start_signal_played")]
    )
