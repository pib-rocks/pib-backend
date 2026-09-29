#!/usr/bin/env python3
"""Check that every models/manifest.yaml entry exists and matches sha256."""

from __future__ import annotations

import hashlib
import sys
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent
MANIFEST_PATH = MODELS_DIR / "manifest.yaml"


def _unquote(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def _parse_scalar(value: str):
    value = _unquote(value.strip())
    if value in {"true", "True"}:
        return True
    if value in {"false", "False"}:
        return False
    if value in {"null", "None", "~", ""}:
        return None
    if value.isdigit() or (value.startswith("-") and value[1:].isdigit()):
        return int(value)
    return value


def load_manifest(path: Path) -> dict:
    """Load the registry YAML (PyYAML if present, else a small subset parser)."""
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or "models" not in data:
            raise ValueError("manifest.yaml must be a mapping with a 'models' list")
        return data
    except ImportError:
        pass

    models: list[dict] = []
    current: dict | None = None
    in_models = False
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        line = raw_line.rstrip()
        if line.startswith("models:"):
            in_models = True
            continue
        if not in_models:
            continue
        if line.startswith("- "):
            if current:
                models.append(current)
            current = {}
            rest = line[2:]
            if ":" in rest:
                key, val = rest.split(":", 1)
                current[key.strip()] = _parse_scalar(val)
            continue
        if current is None:
            continue
        if line.startswith("  ") and ":" in line:
            key, val = line.strip().split(":", 1)
            current[key.strip()] = _parse_scalar(val)
    if current:
        models.append(current)
    if not models:
        raise ValueError("no models parsed from manifest.yaml (install PyYAML?)")
    return {"models": models}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    if not MANIFEST_PATH.is_file():
        print(f"FAIL  manifest missing: {MANIFEST_PATH}", file=sys.stderr)
        return 1

    try:
        manifest = load_manifest(MANIFEST_PATH)
    except Exception as exc:  # noqa: BLE001 — report parse errors clearly
        print(f"FAIL  cannot read {MANIFEST_PATH}: {exc}", file=sys.stderr)
        return 1

    entries = manifest.get("models") or []
    if not entries:
        print("FAIL  manifest has no models", file=sys.stderr)
        return 1

    failed = 0
    for entry in entries:
        model_id = entry.get("model_id", "<missing model_id>")
        rel = entry.get("file")
        expected_sha = str(entry.get("sha256") or "")
        expected_size = entry.get("size_bytes")

        if not rel:
            print(f"FAIL  {model_id}: no 'file' in manifest")
            failed += 1
            continue

        blob_path = MODELS_DIR / rel
        if not blob_path.is_file():
            print(f"FAIL  {model_id}: missing file {blob_path}")
            failed += 1
            continue

        actual_size = blob_path.stat().st_size
        actual_sha = sha256_file(blob_path)
        problems = []
        if expected_size is not None and actual_size != int(expected_size):
            problems.append(f"size {actual_size} != {expected_size}")
        if expected_sha and actual_sha != expected_sha:
            problems.append(f"sha256 {actual_sha} != {expected_sha}")
        if problems:
            print(f"FAIL  {model_id}: {'; '.join(problems)}")
            failed += 1
        else:
            print(f"OK    {model_id}  {actual_size} bytes  {actual_sha}")

    print()
    total = len(entries)
    ok = total - failed
    print(f"summary: {ok}/{total} OK, {failed} FAIL")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
