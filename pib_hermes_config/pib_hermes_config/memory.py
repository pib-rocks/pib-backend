"""Bounded MEMORY.md for one Hermes profile (concept section 6.3).

Hermes stores experience as entries joined by a section sign. That file has
no size limit here. A periodic pass folds entries that are no longer recent
into one stable section and leaves the recent entries verbatim. Character
(SOUL) is a different file; this module does not read or write it.

The summary itself comes from the caller. This module does not invent one.
"""

from __future__ import annotations

import logging
import os
import tempfile
import time
from contextlib import contextmanager
from dataclasses import dataclass

from pib_hermes_config import align_profile_ownership, profile_dir_for

logger = logging.getLogger(__name__)

# Same delimiter as Hermes tools/memory_tool.py. Splitting on the sign alone
# would cut an entry that contains the sign in its text.
ENTRY_DELIMITER = "\n§\n"
STABLE_HEADING = "## Stable"
CHARACTER_LABEL = "Character (SOUL)"
EXPERIENCE_LABEL = "Experience (MEMORY)"
MEMORY_FILENAME = "MEMORY.md"

# How many trailing entries stay word for word. Older ones are the stable section.
RECENT_ENTRY_COUNT = 8
# How often a profile is consolidated. The voice assistant checks more often
# and uses the stamp below, so a restart does not reset the period.
CONSOLIDATION_INTERVAL_SECONDS = 24 * 60 * 60
CONSOLIDATION_CHECK_SECONDS = 60 * 60

STAMP_NAME = ".last_consolidation"


@dataclass(frozen=True)
class ConsolidationResult:
    """Outcome of one consolidation attempt. ``text`` is what should be stored."""

    text: str
    action: str


def memory_path_for(personality_id: str) -> str:
    """Absolute path of the MEMORY.md belonging to one personality."""
    return os.path.join(profile_dir_for(personality_id), "memories", MEMORY_FILENAME)


def read_memory(personality_id: str) -> str:
    """Experience text, or '' when the profile has no memory file yet."""
    path = memory_path_for(personality_id)
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read()
    except FileNotFoundError:
        return ""
    except OSError as exc:
        logger.warning("could not read %s: %s", path, exc)
        return ""


def memory_size(personality_id: str) -> int:
    """Character count of MEMORY.md. Missing file is zero."""
    return len(read_memory(personality_id))


def write_memory(personality_id: str, text: str) -> None:
    """Replace MEMORY.md. Does not touch SOUL.md."""
    path = memory_path_for(personality_id)
    with _file_lock(path):
        _atomic_write(path, text)
    align_profile_ownership(profile_dir_for(personality_id))


def parse_entries(text: str) -> list[str]:
    """Entries the way Hermes reads them. Empty pieces are dropped."""
    if not text or not text.strip():
        return []
    return [entry.strip() for entry in text.split(ENTRY_DELIMITER) if entry.strip()]


def consolidate_text(
    text: str,
    summarize,
    *,
    keep_recent: int = RECENT_ENTRY_COUNT,
) -> ConsolidationResult:
    """Fold old entries into one stable section. Recent entries stay verbatim.

    ``summarize`` receives only the old material and returns a shorter
    replacement, or ``None`` when it cannot. A result that is not shorter
    than the file is refused, so a failed summary cannot grow the memory.
    """
    stable, old, recent = _split_entries(text, keep_recent)
    if not old:
        return ConsolidationResult(text, "unchanged")
    try:
        summary = summarize(_old_material(stable, old))
    except Exception:
        logger.warning("memory summary failed", exc_info=True)
        return ConsolidationResult(text, "blocked")
    cleaned = _clean_summary(summary)
    if not cleaned:
        return ConsolidationResult(text, "blocked")
    rendered = _render(cleaned, recent)
    if len(rendered) >= len(text):
        return ConsolidationResult(text, "unchanged")
    parsed = parse_entries(rendered)
    tail = parsed[-len(recent) :] if recent else []
    if tail != recent:
        return ConsolidationResult(text, "blocked")
    return ConsolidationResult(rendered, "consolidated")


def consolidate_memory_file(
    path: str,
    summarize,
    *,
    now: float,
    interval: float = CONSOLIDATION_INTERVAL_SECONDS,
    keep_recent: int = RECENT_ENTRY_COUNT,
) -> str:
    """Consolidate one file when its last pass is older than ``interval``.

    Returns ``skipped``, ``unchanged``, ``consolidated``, or ``blocked``.
    ``blocked`` leaves the file and the stamp alone so the next check retries.
    """
    with _file_lock(path):
        if not _is_due(path, now, interval):
            return "skipped"
        original = _read_path(path)

    result = consolidate_text(original, summarize, keep_recent=keep_recent)
    if result.action == "blocked":
        return "blocked"

    with _file_lock(path):
        if _read_path(path) != original:
            return "blocked"
        if result.action == "consolidated":
            _atomic_write(path, result.text)
            align_profile_ownership(_profile_dir_of(path))
        _write_stamp(path, now)
    return result.action


def consolidate_due_profiles(
    summarize,
    *,
    now: float | None = None,
    profiles_root: str | None = None,
    interval: float = CONSOLIDATION_INTERVAL_SECONDS,
    keep_recent: int = RECENT_ENTRY_COUNT,
) -> dict[str, str]:
    """Run the due pass on every profile MEMORY.md under the profiles root."""
    moment = time.time() if now is None else now
    results: dict[str, str] = {}
    for path in iter_memory_files(profiles_root):
        results[path] = consolidate_memory_file(
            path,
            summarize,
            now=moment,
            interval=interval,
            keep_recent=keep_recent,
        )
    return results


def iter_memory_files(profiles_root: str | None = None) -> list[str]:
    """MEMORY.md paths that exist. The root defaults to the shared profiles dir."""
    from pib_hermes_config import profiles_dir

    root = profiles_root if profiles_root is not None else profiles_dir()
    if not os.path.isdir(root):
        return []
    found: list[str] = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "memories", MEMORY_FILENAME)
        if os.path.isfile(path):
            found.append(path)
    return found


def _split_entries(text: str, keep_recent: int) -> tuple[str, list[str], list[str]]:
    entries = parse_entries(text)
    stable = ""
    journal = entries
    if entries and _is_stable_entry(entries[0]):
        stable = _stable_body(entries[0])
        journal = entries[1:]
    keep = max(0, int(keep_recent))
    if keep >= len(journal):
        return stable, [], journal
    if keep == 0:
        return stable, journal, []
    return stable, journal[:-keep], journal[-keep:]


def _is_stable_entry(entry: str) -> bool:
    return entry == STABLE_HEADING or entry.startswith(STABLE_HEADING + "\n")


def _stable_body(entry: str) -> str:
    if entry == STABLE_HEADING:
        return ""
    return entry[len(STABLE_HEADING) + 1 :]


def _old_material(stable: str, old: list[str]) -> str:
    parts: list[str] = []
    if stable.strip():
        parts.append(stable.strip())
    parts.extend(old)
    return "\n\n".join(parts)


def _clean_summary(summary: object) -> str:
    if not isinstance(summary, str):
        return ""
    text = summary.strip()
    text = text.replace(ENTRY_DELIMITER, "\n")
    text = text.replace("§", "")
    if text == STABLE_HEADING:
        return ""
    if text.startswith(STABLE_HEADING + "\n"):
        text = text[len(STABLE_HEADING) + 1 :]
    return text.strip()


def _render(stable: str, recent: list[str]) -> str:
    entries: list[str] = []
    if stable.strip():
        entries.append(STABLE_HEADING + "\n" + stable.strip())
    entries.extend(recent)
    if not entries:
        return ""
    return ENTRY_DELIMITER.join(entries)


def _stamp_path(memory_path: str) -> str:
    return os.path.join(os.path.dirname(memory_path), STAMP_NAME)


def _is_due(memory_path: str, now: float, interval: float) -> bool:
    stamp = _stamp_path(memory_path)
    try:
        raw = open(stamp, encoding="utf-8").read().strip()
    except FileNotFoundError:
        return True
    except OSError:
        return True
    try:
        last = float(raw)
    except ValueError:
        return True
    return now - last >= interval


def _write_stamp(memory_path: str, now: float) -> None:
    _atomic_write(_stamp_path(memory_path), str(int(now)))


def _read_path(path: str) -> str:
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read()
    except FileNotFoundError:
        return ""
    except OSError as exc:
        logger.warning("could not read %s: %s", path, exc)
        return ""


def _atomic_write(path: str, content: str) -> None:
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=directory, prefix=".mem_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _profile_dir_of(memory_path: str) -> str:
    return os.path.dirname(os.path.dirname(memory_path))


@contextmanager
def _file_lock(path: str):
    """Exclusive lock shared with Hermes: ``MEMORY.md.lock``."""
    import fcntl

    lock_path = path + ".lock"
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    handle = open(lock_path, "a+", encoding="utf-8")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield
    finally:
        try:
            fcntl.flock(handle, fcntl.LOCK_UN)
        except OSError:
            pass
        handle.close()
