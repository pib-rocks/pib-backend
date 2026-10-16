"""Turn-taking for a running assistant (concept section 6.1).

The pause threshold already lives on the personality. A running assistant
applies the value it just read, so an edit is used on the next recording.
Speech is handed to the synthesizer one clause at a time while the answer
is still arriving. A slow first token is covered only by text stored on
that personality. There is no built-in phrase.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

DEFAULT_FIRST_TOKEN_BUDGET_MS = 700
DEFAULT_FALLBACK_TOKEN_LIMIT = 25


def transcript_opens_turn(text: object) -> bool:
    """Whether a transcript may open a chat turn (PC-40).

    A window nobody spoke into transcribes to an empty string. Sending that on
    makes the robot answer itself: the model replies to an empty message with a
    filler sentence and that answer is spoken - measured on the test robot as
    seven self-triggered turns with empty transcripts in four minutes, each
    answered with 6.5-55 s of audio. So only text opens a turn; anything that
    is not a non-blank string is "nothing was said".
    """
    return isinstance(text, str) and bool(text.strip())


_CLAUSE_MARK = re.compile(r"[,;:.!?\n]")
_PROGRAM_BLOCK = re.compile(r"<pib-program>.*?</pib-program>", re.DOTALL)
_OPEN_PROGRAM = re.compile(r"<pib-program>.*\Z", re.DOTALL)
_WHITESPACE = re.compile(r"\s+")
_WORD = re.compile(r"\S+")
_ABBREVIATIONS = frozenset(
    {
        "dr",
        "prof",
        "mr",
        "mrs",
        "ms",
        "nr",
        "ca",
        "bzw",
        "etc",
        "vs",
        "st",
        "z.b",
        "d.h",
        "u.a",
        "b",
        "h",
        "a",
    }
)


def fallback_token_limit() -> int:
    """Maximum words before an unpunctuated clause flushes at the last space."""
    override = os.environ.get("PIB_TTS_FALLBACK_TOKENS", "").strip()
    if override:
        try:
            value = int(override)
            if value > 0:
                return value
        except ValueError:
            pass
    return DEFAULT_FALLBACK_TOKEN_LIMIT


def first_token_budget_ms() -> int:
    """Budget for the first token, from voice/whisper-model.yaml when present."""
    override = os.environ.get("PIB_FIRST_TOKEN_BUDGET_MS")
    if override:
        return int(override)
    path = _whisper_model_yaml()
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if stripped.startswith("first_token_budget_ms:"):
                return int(stripped.split(":", 1)[1].strip())
    return DEFAULT_FIRST_TOKEN_BUDGET_MS


def pause_threshold_now(cached: float, refreshed: float | None) -> float:
    """The silence limit the next recording uses.

    A failed refresh keeps the value the assistant already has. A fetched
    number replaces it, which is how an edit reaches a running assistant.
    """
    if refreshed is None:
        return float(cached)
    return float(refreshed)


def authored_filler(
    filler: str | None,
    *,
    elapsed_ms: float,
    budget_ms: int,
    first_token_seen: bool,
) -> str | None:
    """Personality text to speak while the first token is still late.

    Returns None when the token already arrived, the wait is inside the
    budget, or the personality has no text of its own. Never substitutes
    a phrase.
    """
    if first_token_seen or elapsed_ms < budget_ms:
        return None
    if not isinstance(filler, str):
        return None
    text = filler.strip()
    return text or None


def normalize_thinking_filler(value: object) -> str | None:
    """Store authored filler text, or nothing. Blank is nothing."""
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def speakable_answer(text: str) -> str:
    """Answer text with program blocks removed so they are not spoken."""
    without = _PROGRAM_BLOCK.sub(" ", text)
    return _OPEN_PROGRAM.sub(" ", without)


def completed_clauses(
    text: str, *, max_tokens: int | None = None
) -> tuple[list[str], str]:
    """Clauses that end on punctuation or token-limit fallback, and the unfinished tail.

    A decimal point inside a number or an abbreviation period is not a boundary.
    An ellipsis stays with the clause it closes. When an unpunctuated span exceeds
    ``max_tokens`` words (default 25), it flushes at the whitespace after the
    ``max_tokens``-th word so a long clause without punctuation cannot stall first audio.
    """
    limit = fallback_token_limit() if max_tokens is None else int(max_tokens)
    clauses: list[str] = []
    start = 0
    for match in _CLAUSE_MARK.finditer(text):
        index = match.start()
        mark = match.group()
        if mark == "." and _is_decimal_point(text, index):
            continue
        if mark == "." and index + 1 < len(text) and text[index + 1] == ".":
            continue
        if mark == "." and _is_abbreviation(text, index):
            continue
        if mark == ":" and _is_non_boundary_colon(text, index):
            continue
        segment = text[start : match.end()]
        if limit > 0:
            prefix = text[start:index]
            words = list(_WORD.finditer(prefix))
            while len(words) > limit:
                split_at = words[limit - 1].end()
                piece = prefix[:split_at].strip()
                if piece:
                    clauses.append(piece)
                prefix = prefix[split_at:]
                words = list(_WORD.finditer(prefix))
            segment = prefix + mark
        piece = segment.strip()
        if piece:
            clauses.append(piece)
        start = match.end()

    rest = text[start:]
    if limit > 0 and rest:
        fallback_clauses, rest = _split_unpunctuated_fallback(rest, limit)
        clauses.extend(fallback_clauses)
    return clauses, rest


def _split_unpunctuated_fallback(text: str, limit: int) -> tuple[list[str], str]:
    """Flush unpunctuated text at the whitespace boundary after ``limit`` words."""
    clauses: list[str] = []
    rest = text
    while limit > 0 and rest:
        words = list(_WORD.finditer(rest))
        if len(words) < limit:
            break
        nth_end = words[limit - 1].end()
        has_following_space = len(words) > limit or (
            nth_end < len(rest) and rest[nth_end:].isspace()
        )
        if not has_following_space:
            break
        piece = rest[:nth_end].strip()
        if piece:
            clauses.append(piece)
        rest = rest[nth_end:]
    return clauses, rest


def clauses_to_synthesize(text: str, *, max_tokens: int | None = None) -> list[str]:
    """Every piece of text to synthesize, first clause first.

    The unfinished tail is included so the end of an answer is not dropped.
    An empty string yields nothing.
    """
    clauses, rest = completed_clauses(speakable_answer(text), max_tokens=max_tokens)
    tail = rest.strip()
    if tail:
        clauses.append(tail)
    return clauses


def unpublished_clauses(
    answer: str, published: list[str], *, max_tokens: int | None = None
) -> list[str]:
    """Clauses in answer whose text was not already published."""
    covered = _covered_norms(published, max_tokens=max_tokens)
    fresh: list[str] = []
    for clause in completed_clauses(speakable_answer(answer), max_tokens=max_tokens)[0]:
        key = _norm(clause)
        if not key or key in covered:
            continue
        fresh.append(clause)
        covered.add(key)
    return fresh


def _speech_emitted_prefix(published: list[str]) -> str:
    return "".join(item for item in published if isinstance(item, str))


def _clause_spans(
    speakable: str, *, max_tokens: int | None = None
) -> list[tuple[int, int]]:
    clauses = clauses_to_synthesize(speakable, max_tokens=max_tokens)
    spans: list[tuple[int, int]] = []
    cursor = 0
    for clause in clauses:
        index = speakable.find(clause, cursor)
        if index < 0:
            index = cursor
        spans.append((index, index + len(clause)))
        cursor = index + len(clause)
    return spans


def next_speech_delta(
    answer: str, published: list[str], *, max_tokens: int | None = None
) -> str:
    """Next speakable substring not yet present in published deltas."""
    speakable = speakable_answer(answer)
    if not speakable:
        return ""
    emitted = _speech_emitted_prefix(published)
    if not speakable.startswith(emitted):
        return ""
    emitted_len = len(emitted)
    if emitted_len >= len(speakable):
        return ""
    for _start, end in _clause_spans(speakable, max_tokens=max_tokens):
        if end > emitted_len:
            return speakable[emitted_len:end]
    return speakable[emitted_len:]


def speech_result_tail(
    answer: str, published: list[str], *, max_tokens: int | None = None
) -> str:
    """Speakable text not yet sent as action feedback (Chat.Result delta)."""
    speakable = speakable_answer(answer)
    emitted = _speech_emitted_prefix(published)
    if not speakable.startswith(emitted):
        return ""
    return speakable[len(emitted) :]


class StreamingSpeech:
    """Consume delta-only speech payloads for one assistant answer."""

    def __init__(self, *, max_tokens: int | None = None) -> None:
        self._buffer = ""
        self._emitted_clause_count = 0
        self._finalized = False
        self.max_tokens = (
            fallback_token_limit() if max_tokens is None else int(max_tokens)
        )

    @property
    def finalized(self) -> bool:
        return self._finalized

    def take(self, delta: str, is_final: bool) -> list[str]:
        """Clauses to synthesize from this delta-only payload.

        Each call receives only text not yet emitted on the wire. Repeated
        final delivery after finalization is ignored; intentional repeated
        phrases in distinct deltas are spoken again.
        """
        if self._finalized:
            return []
        if not isinstance(delta, str):
            delta = ""
        if is_final:
            self._finalized = True

        # Buffer every delta, including whitespace-only separators: the wire
        # splits one answer across payloads, and dropping a lone " " would glue
        # the neighbouring words together in the synthesized text.
        self._buffer += delta
        if not self._buffer.strip():
            return []

        clauses, rest = completed_clauses(
            speakable_answer(self._buffer), max_tokens=self.max_tokens
        )
        if (
            not is_final
            and self.max_tokens > 0
            and rest.strip()
            and len(_WORD.findall(rest)) == self.max_tokens
            and len(_WORD.findall(delta)) >= self.max_tokens
        ):
            clauses = list(clauses)
            clauses.append(rest.strip())
            rest = ""

        fresh = clauses[self._emitted_clause_count :]
        self._emitted_clause_count = len(clauses)
        if is_final:
            tail = rest.strip()
            if tail:
                fresh = list(fresh)
                fresh.append(tail)
        return list(fresh)


def _whisper_model_yaml() -> Path:
    return Path(__file__).resolve().parents[2] / "voice" / "whisper-model.yaml"


def _is_decimal_point(text: str, index: int) -> bool:
    return (
        index > 0
        and index + 1 < len(text)
        and text[index - 1].isdigit()
        and text[index + 1].isdigit()
    )


def _is_abbreviation(text: str, index: int) -> bool:
    """True when the period at ``index`` closes a known abbreviation (e.g. z.B., Dr.)."""
    prefix = text[:index]
    word_match = re.search(r"([A-Za-zÄÖÜäöüß]+(?:\.[A-Za-zÄÖÜäöüß]+)*)\Z", prefix)
    if word_match is None:
        return False
    token = word_match.group(1).lower()
    if token in _ABBREVIATIONS:
        # Single-letter initials/abbreviations like 'z.B.' where the next char is 'B.'
        if len(token) == 1 and token not in {"b", "h", "a"}:
            return False
        if len(token) == 1:
            # Only treat single 'b'/'h'/'a' as abbreviation when preceded by 'z.'/'d.'/'u.'
            prev = prefix[: word_match.start()].rstrip()
            if not prev.endswith("."):
                return False
        return True
    # Multi-part abbreviation whose second letter is immediately after this period (e.g. the 'z.' in 'z.B.')
    if (
        len(token) == 1
        and token in {"z", "d", "u"}
        and index + 2 < len(text)
        and text[index + 1].isalpha()
        and text[index + 2] == "."
    ):
        return True
    return False


def _is_non_boundary_colon(text: str, index: int) -> bool:
    """True when the colon at ``index`` is inside a time (12:30) or URL (http://)."""
    if (
        index > 0
        and index + 1 < len(text)
        and text[index - 1].isdigit()
        and text[index + 1].isdigit()
    ):
        return True
    if index + 2 < len(text) and text[index + 1 : index + 3] == "//":
        return True
    return False


def _norm(text: str) -> str:
    return _WHITESPACE.sub(" ", text).strip()


def _extends(text: str, prefix: str) -> bool:
    """True when text continues prefix, not when prefix is a word fragment."""
    if not prefix:
        return True
    if text == prefix:
        return True
    if not text.startswith(prefix):
        return False
    return text[len(prefix)] in " \n\t,;:.!?"


def _already_spoken(spoken: str, candidate: str) -> bool:
    """True when candidate is the spoken prefix, not when it adds a new clause."""
    if not spoken:
        return False
    norm = _norm(candidate)
    return norm == spoken or _extends(spoken, norm)


def _covered_norms(published: list[str], *, max_tokens: int | None = None) -> set[str]:
    covered: set[str] = set()
    for item in published:
        if not isinstance(item, str):
            continue
        covered.add(_norm(item))
        for clause in completed_clauses(speakable_answer(item), max_tokens=max_tokens)[
            0
        ]:
            covered.add(_norm(clause))
    covered.discard("")
    return covered
