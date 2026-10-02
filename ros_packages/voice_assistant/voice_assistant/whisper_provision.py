"""Place a local faster-whisper model into /data/voice/models/whisper/.

The copy reads files that are already on disk. It does not download.
models/manifest.yaml is the camera blob list and is not consulted.
"""

from __future__ import annotations

import hashlib
import os
import shutil
from pathlib import Path

WEIGHTS_NAME = "model.bin"
DEFAULT_DESTINATION = Path("/data/voice/models/whisper")
VENDORED_RELATIVE = Path("voice") / "whisper"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_tree(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for item in source.iterdir():
        if not item.is_file():
            continue
        temporary = destination / f".{item.name}.tmp"
        shutil.copy2(item, temporary)
        temporary.replace(destination / item.name)


def provision_whisper_model(source: Path, destination: Path) -> str:
    """Copy ``source`` into ``destination``.

    Returns ``placed``, ``already_current``, or ``missing``. A missing
    source is not fetched. ``source`` may be the flat model directory
    (``model.bin`` beside the other files) or a parent of size
    directories such as ``small/``.
    """
    source = Path(source)
    destination = Path(destination)
    if (source / WEIGHTS_NAME).is_file():
        return _place_one(source, destination)
    placed = False
    current = False
    found = False
    if source.is_dir():
        for child in sorted(source.iterdir()):
            if not child.is_dir() or not (child / WEIGHTS_NAME).is_file():
                continue
            found = True
            result = _place_one(child, destination / child.name)
            placed = placed or result == "placed"
            current = current or result == "already_current"
    if not found:
        return "missing"
    if placed:
        return "placed"
    return "already_current"


def _place_one(source: Path, destination: Path) -> str:
    weights = source / WEIGHTS_NAME
    dest_weights = destination / WEIGHTS_NAME
    if dest_weights.is_file() and _sha256(dest_weights) == _sha256(weights):
        return "already_current"
    _copy_tree(source, destination)
    return "placed"


def provision_from_repo(repo_root: Path, destination: Path | None = None) -> str:
    """Copy the vendored tree if it contains weights. Never downloads."""
    dest = (
        Path(destination)
        if destination is not None
        else Path(os.environ.get("WHISPER_MODEL_PATH", str(DEFAULT_DESTINATION)))
    )
    return provision_whisper_model(Path(repo_root) / VENDORED_RELATIVE, dest)
