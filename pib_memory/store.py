"""Memory store read/write API backed by SQLite."""

from __future__ import annotations

import json
import os
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import db


class MissingEvidenceRefError(ValueError):
    """Raised when an episode is written without an evidence reference."""


@dataclass
class MemoryStore:
    """SQLite-backed episodic memory. Cloud sync is out of scope; swap backend here later."""

    path: str | Path

    def __post_init__(self) -> None:
        self._conn = db.connect(self.path)
        db.init_schema(self._conn)

    @classmethod
    def default(cls) -> MemoryStore:
        env_path = os.environ.get("PIB_MEMORY_DB_PATH")
        if env_path:
            return cls(env_path)
        return cls(Path.home() / ".pib" / "episodic_memory.db")

    def close(self) -> None:
        self._conn.close()

    def write_episode(
        self,
        *,
        ts_ms: int | None = None,
        place_id: str | None = None,
        participants: list[str] | None = None,
        task: str | None = None,
        outcome: str | None = None,
        failure_reason: str | None = None,
        evidence_ref: str,
        consent_scope: str | None = None,
        created_ms: int | None = None,
    ) -> int:
        ref = (evidence_ref or "").strip()
        if not ref:
            raise MissingEvidenceRefError(
                "evidence_ref is required; episodic memory is not stored without evidence"
            )
        now = int(time.time() * 1000)
        ts = ts_ms if ts_ms is not None else now
        created = created_ms if created_ms is not None else now
        participants_json = json.dumps(participants or [])
        cur = self._conn.execute(
            """
            INSERT INTO episodes (
                ts_ms, place_id, participants_json, task, outcome,
                failure_reason, evidence_ref, consent_scope, created_ms
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                ts,
                place_id,
                participants_json,
                task,
                outcome,
                failure_reason,
                ref,
                consent_scope,
                created,
            ),
        )
        self._conn.commit()
        return int(cur.lastrowid)

    def upsert_object(
        self,
        *,
        object_id: str,
        label: str | None = None,
        place_id: str | None = None,
        last_seen_ms: int | None = None,
        confidence: float = 0.0,
        frame_ref: str | None = None,
    ) -> int:
        oid = object_id.strip()
        if not oid:
            raise ValueError("object_id must not be empty")
        now = int(time.time() * 1000)
        seen = last_seen_ms if last_seen_ms is not None else now
        self._conn.execute(
            """
            INSERT INTO object_memory (
                object_id, label, place_id, last_seen_ms, confidence, frame_ref
            ) VALUES (?, ?, ?, ?, ?, ?)
            ON CONFLICT(object_id) DO UPDATE SET
                label=excluded.label,
                place_id=excluded.place_id,
                last_seen_ms=excluded.last_seen_ms,
                confidence=excluded.confidence,
                frame_ref=excluded.frame_ref
            """,
            (oid, label, place_id, seen, confidence, frame_ref),
        )
        self._conn.commit()
        row = self._conn.execute(
            "SELECT id FROM object_memory WHERE object_id = ?", (oid,)
        ).fetchone()
        return int(row["id"])

    def upsert_person(
        self,
        *,
        display_name: str,
        embedding_ref: str | None = None,
        consent: str | None = None,
        preferences: dict[str, Any] | None = None,
        created_ms: int | None = None,
    ) -> int:
        name = display_name.strip()
        if not name:
            raise ValueError("display_name must not be empty")
        now = int(time.time() * 1000)
        created = created_ms if created_ms is not None else now
        prefs_json = json.dumps(preferences or {})
        self._conn.execute(
            """
            INSERT INTO person_memory (
                display_name, embedding_ref, consent, preferences_json, created_ms
            ) VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(display_name) DO UPDATE SET
                embedding_ref=excluded.embedding_ref,
                consent=excluded.consent,
                preferences_json=excluded.preferences_json
            """,
            (name, embedding_ref, consent, prefs_json, created),
        )
        self._conn.commit()
        row = self._conn.execute(
            "SELECT id FROM person_memory WHERE display_name = ?", (name,)
        ).fetchone()
        return int(row["id"])

    def delete_person(self, person_id_or_name: str) -> None:
        key = person_id_or_name.strip()
        if not key:
            return
        person_row = None
        if key.isdigit():
            person_row = self._conn.execute(
                "SELECT id, display_name FROM person_memory WHERE id = ?", (int(key),)
            ).fetchone()
        if person_row is None:
            person_row = self._conn.execute(
                "SELECT id, display_name FROM person_memory WHERE display_name = ?",
                (key,),
            ).fetchone()
        if person_row is None:
            return
        display_name = person_row["display_name"]
        person_id = int(person_row["id"])
        episodes = self._conn.execute(
            "SELECT id, participants_json FROM episodes"
        ).fetchall()
        for ep in episodes:
            try:
                participants = json.loads(ep["participants_json"] or "[]")
            except json.JSONDecodeError:
                participants = []
            if not isinstance(participants, list):
                participants = []
            names = {str(p) for p in participants}
            if display_name in names or str(person_id) in names:
                self._conn.execute("DELETE FROM episodes WHERE id = ?", (ep["id"],))
        self._conn.execute("DELETE FROM person_memory WHERE id = ?", (person_id,))
        self._conn.commit()

    def delete_place(self, place_id: str) -> None:
        pid = place_id.strip()
        if not pid:
            return
        self._conn.execute("DELETE FROM episodes WHERE place_id = ?", (pid,))
        self._conn.execute("DELETE FROM object_memory WHERE place_id = ?", (pid,))
        self._conn.commit()

    def delete_all(self) -> None:
        self._conn.execute("DELETE FROM episodes")
        self._conn.execute("DELETE FROM object_memory")
        self._conn.execute("DELETE FROM person_memory")
        self._conn.commit()

    def export(self) -> dict[str, Any]:
        episodes = [
            dict(row)
            for row in self._conn.execute("SELECT * FROM episodes ORDER BY id")
        ]
        objects = [
            dict(row)
            for row in self._conn.execute("SELECT * FROM object_memory ORDER BY id")
        ]
        persons = [
            dict(row)
            for row in self._conn.execute("SELECT * FROM person_memory ORDER BY id")
        ]
        meta = {
            row["key"]: row["value"]
            for row in self._conn.execute("SELECT key, value FROM meta")
        }
        return {
            "meta": meta,
            "episodes": episodes,
            "object_memory": objects,
            "person_memory": persons,
        }

    def purge(self, older_than_ms: int) -> None:
        self._conn.execute("DELETE FROM episodes WHERE ts_ms < ?", (older_than_ms,))
        self._conn.execute(
            "DELETE FROM object_memory WHERE last_seen_ms < ?", (older_than_ms,)
        )
        self._conn.execute(
            "DELETE FROM person_memory WHERE created_ms < ?", (older_than_ms,)
        )
        self._conn.commit()

    def connection(self) -> sqlite3.Connection:
        return self._conn
