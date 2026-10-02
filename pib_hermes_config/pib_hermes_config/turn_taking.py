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

_CLAUSE_MARK = re.compile(r"[,;:.!?]")
_PROGRAM_BLOCK = re.compile(r"<pib-program>.*?</pib-program>", re.DOTALL)
_OPEN_PROGRAM = re.compile(r"<pib-program>.*\Z", re.DOTALL)
_WHITESPACE = re.compile(r"\s+")


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


def completed_clauses(text: str) -> tuple[list[str], str]:
    """Clauses that end on punctuation, and the unfinished tail.

    A decimal point inside a number is not a boundary. An ellipsis stays
    with the clause it closes.
    """
    clauses: list[str] = []
    start = 0
    for match in _CLAUSE_MARK.finditer(text):
        index = match.start()
        mark = match.group()
        if mark == "." and _is_decimal_point(text, index):
            continue
        if mark == "." and index + 1 < len(text) and text[index + 1] == ".":
            continue
        piece = text[start : match.end()].strip()
        if piece:
            clauses.append(piece)
        start = match.end()
    return clauses, text[start:]


def clauses_to_synthesize(text: str) -> list[str]:
    """Every piece of text to synthesize, first clause first.

    The unfinished tail is included so the end of an answer is not dropped.
    An empty string yields nothing.
    """
    clauses, rest = completed_clauses(speakable_answer(text))
    tail = rest.strip()
    if tail:
        clauses.append(tail)
    return clauses


def unpublished_clauses(answer: str, published: list[str]) -> list[str]:
    """Clauses in answer whose text was not already published."""
    covered = _covered_norms(published)
    fresh: list[str] = []
    for clause in completed_clauses(speakable_answer(answer))[0]:
        key = _norm(clause)
        if not key or key in covered:
            continue
        fresh.append(clause)
        covered.add(key)
    return fresh


class StreamingSpeech:
    """Speak each clause once while later chunks of the same answer arrive."""

    def __init__(self) -> None:
        self.pending = ""
        self._spoken = ""

    def take(self, chunk: str, is_final: bool) -> list[str]:
        """Clauses to hand to the synthesizer for this chunk.

        A later chunk that repeats or extends text already seen does not
        speak that text again. The final chunk also speaks an unfinished tail.
        """
        if not isinstance(chunk, str) or not chunk.strip():
            return []
        merged = self._merge(chunk)
        clauses, rest = completed_clauses(merged)
        fresh: list[str] = []
        consumed = ""
        for clause in clauses:
            candidate = (consumed + " " + clause).strip() if consumed else clause
            consumed = candidate
            if _already_spoken(self._spoken, candidate):
                continue
            fresh.append(clause)
            self._spoken = _norm(candidate)
        if is_final:
            tail = rest.strip()
            whole = _norm(merged)
            if tail and not _already_spoken(self._spoken, whole):
                fresh.append(tail)
                self._spoken = whole
        return fresh

    def _merge(self, chunk: str) -> str:
        incoming = chunk.strip()
        if not self.pending:
            self.pending = incoming
            return self.pending
        pending = _norm(self.pending)
        new = _norm(incoming)
        if pending == new or _extends(pending, new):
            return self.pending
        if _extends(new, pending):
            self.pending = incoming
            return self.pending
        self.pending = f"{self.pending.rstrip()} {incoming}".strip()
        return self.pending


def _whisper_model_yaml() -> Path:
    return Path(__file__).resolve().parents[2] / "voice" / "whisper-model.yaml"


def _is_decimal_point(text: str, index: int) -> bool:
    return (
        index > 0
        and index + 1 < len(text)
        and text[index - 1].isdigit()
        and text[index + 1].isdigit()
    )


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


def _covered_norms(published: list[str]) -> set[str]:
    covered: set[str] = set()
    for item in published:
        if not isinstance(item, str):
            continue
        covered.add(_norm(item))
        for clause in completed_clauses(speakable_answer(item))[0]:
            covered.add(_norm(clause))
    covered.discard("")
    return covered
