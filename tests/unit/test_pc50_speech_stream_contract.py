"""PC-50: producer (_stream_chunks_to_goal) + consumer (StreamingSpeech) contract."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from unittest.mock import MagicMock

from pib_hermes_config.turn_taking import StreamingSpeech

# Load the sibling test module by path: a third-party 'tests' package in
# site-packages shadows the repository's tests/ directory, so the package
# import form fails here and in CI.
_SIBLING = Path(__file__).with_name("test_chat_hermes_routing.py")
_spec = importlib.util.spec_from_file_location("_pc50_chat_hermes_routing", _SIBLING)
assert _spec is not None and _spec.loader is not None
_routing = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _routing
_spec.loader.exec_module(_routing)

TEXT_TYPE_CODE_VISUAL = _routing.TEXT_TYPE_CODE_VISUAL
TEXT_TYPE_SENTENCE = _routing.TEXT_TYPE_SENTENCE
chat_module = _routing.chat_module
chat_node = _routing.chat_node


def _spoken_clauses(chat_node, tokens: list[str]) -> list[str]:
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    speech = StreamingSpeech()
    spoken: list[str] = []

    _prev, _ptype, tail = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-pc50", tokens
    )

    for call in goal_handle.publish_feedback.call_args_list:
        fb = call[0][0]
        if fb.text_type == TEXT_TYPE_SENTENCE:
            spoken.extend(speech.take(fb.text, False))

    spoken.extend(speech.take(tail, True))
    return spoken


def test_pc50_counterexample_two_sentence_partition(chat_node):
    spoken = _spoken_clauses(chat_node, ["One sentence.", " Second sentence."])
    assert spoken == ["One sentence.", "Second sentence."]


def test_pc50_one_sentence_control(chat_node):
    spoken = _spoken_clauses(chat_node, ["Hallo Welt. "])
    assert spoken == ["Hallo Welt."]


def test_pc50_trailing_whitespace_empty_final_tail(chat_node):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    speech = StreamingSpeech()
    _prev, _ptype, tail = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-pc50", ["One sentence. "]
    )
    for call in goal_handle.publish_feedback.call_args_list:
        fb = call[0][0]
        if fb.text_type == TEXT_TYPE_SENTENCE:
            speech.take(fb.text, False)
    assert tail == ""
    assert speech.take(tail, True) == []
    assert speech.finalized


def test_pc50_token_split_boundaries_and_unicode(chat_node):
    spoken = _spoken_clauses(chat_node, ["Grüße! ", "Pi ist 3.14, ", "grob."])
    assert spoken == ["Grüße!", "Pi ist 3.14,", "grob."]


def test_pc50_abbreviation_not_split(chat_node):
    spoken = _spoken_clauses(chat_node, ["Dr. Müller sagt z.B. hallo. "])
    assert spoken == ["Dr. Müller sagt z.B. hallo."]


def test_pc50_repeated_final_is_harmless(chat_node):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    speech = StreamingSpeech()
    chat_node._stream_chunks_to_goal(goal_handle, "chat-pc50", ["Eins. "])
    for call in goal_handle.publish_feedback.call_args_list:
        fb = call[0][0]
        if fb.text_type == TEXT_TYPE_SENTENCE:
            speech.take(fb.text, False)
    assert speech.take("", True) == []
    assert speech.take("", True) == []


def test_pc50_code_visual_with_sentence(chat_node):
    spoken = _spoken_clauses(chat_node, ["Hallo. ", "<pib-program>xml</pib-program>"])
    assert spoken == ["Hallo."]
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    prev, ptype, tail = chat_node._stream_chunks_to_goal(
        goal_handle,
        "chat-pc50",
        ["Hallo. ", "<pib-program>xml</pib-program>"],
    )
    assert prev == "xml"
    assert ptype == TEXT_TYPE_CODE_VISUAL
    assert tail == ""


def test_pc50_persistence_incremental_updates(chat_node):
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    chat_node._stream_chunks_to_goal(
        goal_handle, "chat-pc50", ["Eins. ", "Zwei. ", "Drei."]
    )
    calls = chat_node.create_chat_message.call_args_list
    assert len(calls) == 3
    updates = [call[0][3] for call in calls]
    assert updates == [False, True, True]
    texts = [call[0][1] for call in calls]
    assert texts == ["Eins.", "Zwei.", "Drei."]
    # Action path: one CREATE then UPDATEs that concatenate into the full answer.
    assert " ".join(texts) == "Eins. Zwei. Drei."


def test_pc50_clause_stream_partitions(chat_node):
    spoken = _spoken_clauses(chat_node, ["Sure", ", I can", " help you."])
    assert spoken == ["Sure,", "I can help you."]


def test_pc50_whitespace_separator_delta_is_buffered(chat_node):
    """A lone " " payload must not be dropped: dropping it glues the words.

    Measured defect: with the separator skipped, ["Erster ", "Satz. ",
    "Zweiter ", "Satz."] spoke "Erster Satz. ZweiterSatz.".
    """
    spoken = _spoken_clauses(chat_node, ["Erster ", "Satz. ", "Zweiter ", "Satz."])
    assert spoken == ["Erster Satz.", "Zweiter Satz."]
    assert " ".join(spoken) == "Erster Satz. Zweiter Satz."


def test_pc50_result_tail_carries_only_unemitted_text(chat_node):
    """The Result tail must not repeat text already sent as feedback."""
    goal_handle = MagicMock()
    goal_handle.is_cancel_requested = False
    _prev, _ptype, tail = chat_node._stream_chunks_to_goal(
        goal_handle, "chat-pc50", ["Eins. ", "Zwei."]
    )
    assert tail == ""
    speech = StreamingSpeech()
    spoken: list[str] = []
    for call in goal_handle.publish_feedback.call_args_list:
        fb = call[0][0]
        if fb.text_type == TEXT_TYPE_SENTENCE:
            spoken.extend(speech.take(fb.text, False))
    spoken.extend(speech.take(tail, True))
    assert spoken == ["Eins.", "Zwei."]
