"""Section 6.1: pause threshold, clause speech, and authored filler."""

from pib_hermes_config.turn_taking import (
    StreamingSpeech,
    authored_filler,
    clauses_to_synthesize,
    first_token_budget_ms,
    normalize_thinking_filler,
    pause_threshold_now,
    unpublished_clauses,
)


def test_budget_is_the_whisper_model_figure():
    assert first_token_budget_ms() == 700


def test_pause_threshold_from_a_refresh_replaces_the_cached_value():
    assert pause_threshold_now(0.8, 1.6) == 1.6


def test_pause_threshold_stays_when_the_refresh_fails():
    assert pause_threshold_now(0.8, None) == 0.8


def test_slow_first_token_uses_the_personality_filler():
    spoken = authored_filler(
        " Moment mal. ",
        elapsed_ms=800,
        budget_ms=700,
        first_token_seen=False,
    )
    assert spoken == "Moment mal."


def test_fast_first_token_is_not_covered():
    assert (
        authored_filler(
            "Moment mal.",
            elapsed_ms=100,
            budget_ms=700,
            first_token_seen=False,
        )
        is None
    )


def test_missing_filler_stays_silent():
    assert (
        authored_filler(None, elapsed_ms=5000, budget_ms=700, first_token_seen=False)
        is None
    )
    assert (
        authored_filler("   ", elapsed_ms=5000, budget_ms=700, first_token_seen=False)
        is None
    )
    assert normalize_thinking_filler("  ") is None
    assert normalize_thinking_filler(None) is None


def test_filler_does_not_speak_after_the_token_arrived():
    assert (
        authored_filler(
            "Moment mal.",
            elapsed_ms=5000,
            budget_ms=700,
            first_token_seen=True,
        )
        is None
    )


def test_synthesis_order_starts_at_the_first_clause():
    assert clauses_to_synthesize("Sure, I can help.") == ["Sure,", "I can help."]


def test_decimal_point_is_not_a_clause_boundary():
    assert clauses_to_synthesize("Pi is 3.14, roughly.") == [
        "Pi is 3.14,",
        "roughly.",
    ]


def test_streaming_speech_speaks_a_clause_once():
    speech = StreamingSpeech()
    assert speech.take("Sure, ", False) == ["Sure,"]
    assert speech.take("Sure, I can help.", True) == ["I can help."]


def test_streaming_speech_does_not_repeat_the_first_token():
    speech = StreamingSpeech()
    assert speech.take("Hallo Welt. ", False) == ["Hallo Welt."]
    assert speech.take("Hallo Welt.", True) == []


def test_streaming_speech_appends_the_next_sentence():
    speech = StreamingSpeech()
    assert speech.take("Eins. ", False) == ["Eins."]
    assert speech.take("Zwei.", True) == ["Zwei."]


def test_unpublished_clauses_skip_text_already_sent():
    fresh = unpublished_clauses("Sure, I can help.", ["Sure, I can help."])
    assert fresh == []
    fresh = unpublished_clauses("Sure, I can help.", ["Sure"])
    assert fresh == ["Sure,", "I can help."]
