"""Why a Hermes turn produced no answer, in words the robot can say (PR-2132).

The spoken fallback was one sentence - "that took a moment too long" - for every
failure, including the ones that took 1.3 seconds. A read-only Hermes home, a
rejected provider key and a real timeout all sounded the same, so neither the
user nor a support screenshot could tell them apart.

Two rules shape this module:

* **The reason is classified from what actually failed**, not from the path that
  happened to run: a read-only file system, a full disk, a missing binary, a
  rejected key, an exhausted budget, an unreachable daemon, a real timeout, or a
  stream that ended without a final reply.
* **A sentence names the cause and what can be done**, in the same language the
  robot speaks. The technical detail stays in the metadata and the log: it is
  for the operator, not for the person standing in front of the robot.

Nothing here talks to a provider, a subprocess or ROS: the classifier and the
sentences are pure, so they are testable without a robot.
"""

from __future__ import annotations

import re
from typing import Optional

#: A failure the operator can fix on the robot, told apart from a transport one.
REASON_HOME_READ_ONLY = "hermes_home_read_only"
REASON_DISK_FULL = "disk_full"
REASON_AGENT_MISSING = "hermes_binary_missing"
REASON_AGENT_UNREACHABLE = "hermes_daemon_unreachable"
REASON_PROVIDER_KEY_REJECTED = "provider_key_rejected"
REASON_PROVIDER_BUDGET = "provider_budget_exhausted"
REASON_STREAM_INCOMPLETE = "incomplete_stream"
REASON_TURN_TIMEOUT = "turn_timeout"
REASON_UNKNOWN = "turn_failed"

#: What the robot says. One sentence per cause, no shared catch-all.
SENTENCES: dict[str, str] = {
    REASON_HOME_READ_ONLY: (
        "Ich kann gerade nicht antworten: mein Speicher ist schreibgeschützt, "
        "deshalb kann ich mein Profil nicht speichern. "
        "Bitte prüfe den Speicher des Roboters."
    ),
    REASON_DISK_FULL: (
        "Ich kann gerade nicht antworten: mein Speicher ist voll. "
        "Bitte schaffe Platz auf dem Roboter."
    ),
    REASON_AGENT_MISSING: (
        "Ich kann gerade nicht antworten: mein Antwortprogramm ist nicht "
        "installiert. Bitte installiere Hermes für den Benutzer pib."
    ),
    REASON_AGENT_UNREACHABLE: (
        "Ich kann gerade nicht antworten: mein Antwortdienst ist nicht "
        "erreichbar. Bitte versuche es gleich noch einmal."
    ),
    REASON_PROVIDER_KEY_REJECTED: (
        "Ich kann gerade nicht antworten: der Schlüssel für den Anbieter wird "
        "abgelehnt. Bitte speichere unter System > Keys einen gültigen Schlüssel."
    ),
    REASON_PROVIDER_BUDGET: (
        "Ich kann gerade nicht antworten: das Guthaben für den Anbieter ist "
        "aufgebraucht."
    ),
    REASON_STREAM_INCOMPLETE: (
        "Die Antwort ist unterwegs abgebrochen. Frag mich bitte noch einmal."
    ),
    REASON_TURN_TIMEOUT: (
        "Entschuldige, das hat gerade einen Moment zu lange gedauert. "
        "Frag mich bitte später noch einmal."
    ),
    REASON_UNKNOWN: (
        "Ich kann gerade nicht antworten. Der Grund steht im Protokoll des " "Roboters."
    ),
}

#: Markers in an exception, a stderr line or a daemon reason. Ordered: the first
#: match wins, because a read-only home also mentions "run hermes pm repair".
_MARKER_RULES: tuple[tuple[str, tuple[re.Pattern[str], ...]], ...] = (
    (
        REASON_HOME_READ_ONLY,
        (
            re.compile(r"errno\s*30", re.I),
            re.compile(r"read-?only file system", re.I),
            re.compile(r"read-?only database", re.I),
            re.compile(r"attempt to write a readonly database", re.I),
        ),
    ),
    (
        REASON_DISK_FULL,
        (
            re.compile(r"errno\s*28", re.I),
            re.compile(r"no space left on device", re.I),
        ),
    ),
    (
        REASON_AGENT_MISSING,
        (
            re.compile(r"hermes\b.*\b(not found|no such file)", re.I),
            re.compile(r"command not found:\s*hermes", re.I),
        ),
    ),
    (
        REASON_PROVIDER_BUDGET,
        (
            re.compile(r"budget_exhausted|insufficient_quota|quota exceeded", re.I),
            re.compile(r"\b402\b"),
        ),
    ),
    (
        REASON_PROVIDER_KEY_REJECTED,
        (
            re.compile(r"api[ _-]?key[^\n]{0,20}(invalid|not valid|rejected)", re.I),
            re.compile(r"invalid[ _-]?api[ _-]?key", re.I),
            re.compile(
                r"\b(401|403)\b[^\n]{0,40}(unauthorized|forbidden|rejected)?", re.I
            ),
            re.compile(r"permission denied[^\n]{0,40}api[ _-]?key", re.I),
        ),
    ),
)


def _text(*parts: Optional[object]) -> str:
    return "\n".join(str(part) for part in parts if part)


def classify_turn_failure(
    *,
    reason: Optional[str] = None,
    detail: Optional[str] = None,
    source: Optional[str] = None,
) -> str:
    """Name the cause of a turn that produced no answer.

    ``reason`` is a code a transport already reported (the daemon's own reason or
    the client's ``ConnectionError``/``http_500``); ``detail`` is whatever text
    came with it. A reason the daemon classified itself is kept as is.
    """
    if reason in SENTENCES:
        return reason

    haystack = _text(reason, detail)
    if haystack.strip():
        for candidate, patterns in _MARKER_RULES:
            if any(pattern.search(haystack) for pattern in patterns):
                return candidate

    if source in ("timeout", "turn_timeout"):
        return REASON_TURN_TIMEOUT
    if reason and reason.lower() in ("connectionerror", "connecttimeout"):
        return REASON_AGENT_UNREACHABLE
    if reason and reason.startswith("http_5"):
        return REASON_UNKNOWN
    return REASON_UNKNOWN


def failure_sentence(reason: Optional[str], detail: Optional[str] = None) -> str:
    """The sentence the robot speaks for this cause."""
    classified = classify_turn_failure(reason=reason, detail=detail)
    return SENTENCES.get(classified, SENTENCES[REASON_UNKNOWN])


def failure_detail(
    reason: Optional[str], detail: Optional[str] = None, limit: int = 200
) -> str:
    """One short line for the metadata and the log. Never a credential."""
    text = " ".join(_text(detail or reason).split())
    if len(text) > limit:
        text = text[: limit - 1].rstrip() + "…"
    return text


def failure_log_line(
    reason: Optional[str],
    *,
    chat_id: str = "",
    source: str = "",
    detail: Optional[str] = None,
) -> str:
    """One [PERF_TRACE] line per failed turn, so support greps one pattern."""
    classified = classify_turn_failure(reason=reason, detail=detail)
    line = f"[PERF_TRACE] HERMES_TURN_FAILED chat={chat_id} reason={classified}"
    if source:
        line += f" source={source}"
    short = failure_detail(reason, detail)
    if short:
        line += f' detail="{short}"'
    return line
