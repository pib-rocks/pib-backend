"""Wake-word detection ("Hey, pib!") and per-personality Blockly program hooks (PR-1153)."""

from __future__ import annotations

import re
from typing import Any, Optional

WAKE_WORD_PHRASE = "Hey, pib!"

# Common Whisper / STT phonetic transcriptions for "Hey, pib!" in German and English.
_WAKE_WORD_PATTERNS = (
    r"\b(?:hey|hi|hallo|hello|okay|ok)\s*[,!.?\-]*\s*(?:pib|pip|pep|pipp|peb|piip|bip)\b",
    r"\b(?:heypib|heypip|heypep|hallopib|hipib)\b",
)
_WAKE_WORD_REGEX = re.compile("|".join(_WAKE_WORD_PATTERNS), re.IGNORECASE)


def matches_wake_word(transcript: Optional[str]) -> bool:
    """Return True when ``transcript`` contains the 'Hey, pib!' wake word."""
    if not transcript or not isinstance(transcript, str):
        return False
    cleaned = transcript.strip()
    if not cleaned:
        return False
    return bool(_WAKE_WORD_REGEX.search(cleaned))


def normalize_program_choice(value: Any) -> Optional[str]:
    """Normalize a stored Blockly program number (None or '' -> None)."""
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError("Program reference must be a string or null.")
    trimmed = value.strip()
    if not trimmed:
        return None
    if len(trimmed) > 255:
        raise ValueError("Program reference must not exceed 255 characters.")
    return trimmed
