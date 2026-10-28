"""Copy a local supertonic-3 tree into /data/voice/models/supertone/.

The copy reads files that are already on disk. It does not download.
"""

from __future__ import annotations

import hashlib
import os
import shutil
from pathlib import Path

MARKER_RELATIVE = Path("onnx") / "tts.json"
DEFAULT_DESTINATION = Path("/data/voice/models/supertone")
VENDORED_RELATIVE = Path("voice") / "supertone"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_tree(source: Path, destination: Path) -> None:
    for item in source.rglob("*"):
        if not item.is_file():
            continue
        rel = item.relative_to(source)
        target = destination / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.parent / f".{target.name}.tmp"
        shutil.copy2(item, temporary)
        temporary.replace(target)


def provision_supertone_model(source: Path, destination: Path) -> str:
    """Copy ``source`` into ``destination``.

    Returns ``placed``, ``already_current``, or ``missing``. A missing
    source is not fetched.
    """
    source = Path(source)
    destination = Path(destination)
    if not (source / MARKER_RELATIVE).is_file():
        return "missing"
    dest_marker = destination / MARKER_RELATIVE
    if dest_marker.is_file() and _sha256(dest_marker) == _sha256(
        source / MARKER_RELATIVE
    ):
        return "already_current"
    _copy_tree(source, destination)
    return "placed"


def provision_from_repo(repo_root: Path, destination: Path | None = None) -> str:
    """Copy the vendored tree if it contains weights. Never downloads."""
    dest = (
        Path(destination)
        if destination is not None
        else Path(os.environ.get("SUPERTONE_MODEL_PATH", str(DEFAULT_DESTINATION)))
    )
    return provision_supertone_model(Path(repo_root) / VENDORED_RELATIVE, dest)
