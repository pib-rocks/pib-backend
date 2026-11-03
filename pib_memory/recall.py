"""Bounded recall over episodic, object, and person memory."""

from __future__ import annotations

import json
import re
import time
from datetime import datetime, timezone
from typing import Any

from .store import MemoryStore

_STALE_AGE_MS = 30 * 24 * 60 * 60 * 1000


def _ms_to_date(ts_ms: int) -> str:
    dt = datetime.fromtimestamp(ts_ms / 1000.0, tz=timezone.utc)
    return dt.date().isoformat()


def _tokenize(text: str) -> list[str]:
    return [t for t in re.split(r"\W+", text.lower()) if t]


def _keyword_score(query_tokens: list[str], haystack: str) -> float:
    if not query_tokens:
        return 0.0
    blob = haystack.lower()
    hits = sum(1 for token in query_tokens if token in blob)
    return hits / len(query_tokens)


def recall(
    query: str,
    k: int = 5,
    person: str | None = None,
    place: str | None = None,
    *,
    store: MemoryStore | None = None,
) -> list[dict[str, Any]]:
    """Return up to ``k`` memory records ranked by keyword match and recency."""

    memory = store if store is not None else MemoryStore.default()
    query_tokens = _tokenize(query)
    person_filter = person.strip() if person else None
    place_filter = place.strip() if place else None
    now_ms = int(time.time() * 1000)
    candidates: list[tuple[float, int, dict[str, Any]]] = []

    conn = memory.connection()
    for row in conn.execute("SELECT * FROM episodes"):
        participants_raw = row["participants_json"] or "[]"
        try:
            participants = json.loads(participants_raw)
        except json.JSONDecodeError:
            participants = []
        if person_filter and person_filter not in {str(p) for p in participants}:
            continue
        if place_filter and (row["place_id"] or "") != place_filter:
            continue
        text = " ".join(
            filter(
                None,
                [
                    row["task"],
                    row["outcome"],
                    row["failure_reason"],
                    row["place_id"],
                    participants_raw,
                ],
            )
        )
        kw = _keyword_score(query_tokens, text)
        ts = int(row["ts_ms"])
        recency = ts / max(now_ms, 1)
        score = kw * 10.0 + recency
        confidence = min(1.0, 0.35 + kw * 0.5 + recency * 0.15)
        stale = (now_ms - ts) > _STALE_AGE_MS
        record = {
            "kind": "episode",
            "id": row["id"],
            "date": _ms_to_date(ts),
            "confidence": round(confidence, 4),
            "evidence_ref": row["evidence_ref"],
            "stale": stale,
            "task": row["task"],
            "outcome": row["outcome"],
            "place_id": row["place_id"],
        }
        candidates.append((score, ts, record))

    for row in conn.execute("SELECT * FROM object_memory"):
        if place_filter and (row["place_id"] or "") != place_filter:
            continue
        text = " ".join(filter(None, [row["label"], row["object_id"], row["place_id"]]))
        kw = _keyword_score(query_tokens, text)
        if person_filter and kw == 0.0 and not query_tokens:
            continue
        ts = int(row["last_seen_ms"])
        recency = ts / max(now_ms, 1)
        score = kw * 10.0 + recency
        base_conf = float(row["confidence"] or 0.0)
        confidence = min(1.0, max(base_conf, 0.2) * (0.5 + kw) + recency * 0.1)
        evidence = row["frame_ref"]
        if not evidence:
            continue
        stale = (now_ms - ts) > _STALE_AGE_MS
        record = {
            "kind": "object",
            "id": row["id"],
            "date": _ms_to_date(ts),
            "confidence": round(confidence, 4),
            "evidence_ref": evidence,
            "stale": stale,
            "label": row["label"],
            "object_id": row["object_id"],
            "place_id": row["place_id"],
        }
        candidates.append((score, ts, record))

    for row in conn.execute("SELECT * FROM person_memory"):
        if person_filter and row["display_name"] != person_filter:
            continue
        text = " ".join(
            filter(
                None,
                [row["display_name"], row["preferences_json"], row["embedding_ref"]],
            )
        )
        kw = _keyword_score(query_tokens, text)
        ts = int(row["created_ms"])
        recency = ts / max(now_ms, 1)
        score = kw * 10.0 + recency
        confidence = min(1.0, 0.3 + kw * 0.6 + recency * 0.1)
        evidence = row["embedding_ref"]
        if not evidence:
            continue
        stale = (now_ms - ts) > _STALE_AGE_MS
        record = {
            "kind": "person",
            "id": row["id"],
            "date": _ms_to_date(ts),
            "confidence": round(confidence, 4),
            "evidence_ref": evidence,
            "stale": stale,
            "display_name": row["display_name"],
        }
        candidates.append((score, ts, record))

    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    limit = max(0, int(k))
    return [item[2] for item in candidates[:limit]]
