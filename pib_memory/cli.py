"""CLI for episodic memory maintenance."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .store import MemoryStore


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="pib_memory")
    sub = parser.add_subparsers(dest="command", required=True)

    del_person = sub.add_parser("delete-person", help="Remove a person and their episodes")
    del_person.add_argument("name", help="Person display name or numeric id")

    del_place = sub.add_parser("delete-place", help="Remove memories tied to a place")
    del_place.add_argument("place_id", help="Place identifier")

    export_cmd = sub.add_parser("export", help="Export memory database to JSON")
    export_cmd.add_argument("path", help="Output JSON file path")

    args = parser.parse_args(argv)
    store = MemoryStore.default()

    if args.command == "delete-person":
        store.delete_person(args.name)
        return 0
    if args.command == "delete-place":
        store.delete_place(args.place_id)
        return 0
    if args.command == "export":
        payload = store.export()
        out = Path(args.path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return 0

    parser.error(f"unknown command {args.command!r}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
