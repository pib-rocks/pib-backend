#!/usr/bin/env python3
"""Place Supertone supertonic-3 weights into the voice model store at setup time.

Self-contained data step for setup-pib.sh: standard library plus PyYAML. It does
not import the voice_assistant ROS package.

Sources, in order:

1. the store itself - every pinned file under ``<destination>/`` whose sha256
   matches is left alone;
2. the vendored tree ``<repo>/voice/supertone/`` (preserving relative paths);
3. the pinned files listed under ``download:`` in ``voice/supertone-model.yaml``.

The container engine never downloads (``download_at_first_use`` stays false).

The last stdout line is ``result=<word>`` with one of ``placed``,
``already_current`` or ``failed``.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Dict, Optional

DEFAULT_DESTINATION = Path("/data/voice/models/supertone")
CONFIG_RELATIVE = Path("voice") / "supertone-model.yaml"
VENDORED_RELATIVE = Path("voice") / "supertone"
MARKER_RELATIVE = Path("onnx") / "tts.json"
DOWNLOAD_TIMEOUT_SECONDS = 120
CHUNK_SIZE = 1024 * 1024
USER_AGENT = "pib-setup-supertone-provision/1.0"


class ProvisionError(Exception):
    """A reason the weights could not be placed; printed verbatim to stderr."""


def log(message: str) -> None:
    print(f"supertone model: {message}", file=sys.stderr, flush=True)


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(CHUNK_SIZE), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_config(repo_root: Path) -> dict:
    config_path = repo_root / CONFIG_RELATIVE
    if not config_path.is_file():
        raise ProvisionError(f"configuration not found at {config_path}")
    try:
        import yaml  # python3-yaml
    except ImportError as error:
        raise ProvisionError(
            "PyYAML is not importable (apt-get install python3-yaml)"
        ) from error
    with config_path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    if not isinstance(document, dict):
        raise ProvisionError(f"{config_path} does not hold a mapping")
    return document


def pinned_files(config: dict) -> Dict[str, dict]:
    download = config.get("download")
    if not isinstance(download, dict):
        return {}
    files = download.get("files")
    if not isinstance(files, dict) or not files:
        raise ProvisionError("download.files in voice/supertone-model.yaml is empty")
    for name, entry in files.items():
        if not isinstance(entry, dict) or "sha256" not in entry or "size" not in entry:
            raise ProvisionError(f"download.files.{name} needs sha256 and size")
    marker = str(MARKER_RELATIVE).replace("\\", "/")
    if marker not in files and str(MARKER_RELATIVE) not in files:
        raise ProvisionError(f"download.files does not list {MARKER_RELATIVE}")
    return files


def download_url(config: dict, name: str) -> str:
    download = config["download"]
    repository = str(download.get("repository", "")).rstrip("/")
    revision = str(download.get("revision", "main"))
    if not repository:
        raise ProvisionError("download.repository is not set")
    return f"{repository}/resolve/{revision}/{name}"


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


def _tree_matches_marker(source: Path) -> bool:
    return (source / MARKER_RELATIVE).is_file()


def provision_vendored(vendored: Path, destination: Path) -> Optional[str]:
    if not vendored.is_dir():
        return None
    if not _tree_matches_marker(vendored):
        return None
    marker = destination / MARKER_RELATIVE
    if marker.is_file() and sha256_of(marker) == sha256_of(vendored / MARKER_RELATIVE):
        log(f"{destination} already holds the vendored supertonic-3 tree")
        return "already_current"
    _copy_tree(vendored, destination)
    log(f"copied vendored supertonic-3 tree from {vendored} to {destination}")
    return "placed"


def fetch(url: str, target: Path, expected_sha256: str, expected_size: int) -> None:
    temporary = target.parent / f".{target.name}.tmp"
    target.parent.mkdir(parents=True, exist_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    digest = hashlib.sha256()
    received = 0
    try:
        with (
            urllib.request.urlopen(
                request, timeout=DOWNLOAD_TIMEOUT_SECONDS
            ) as response,
            temporary.open("wb") as handle,
        ):
            for chunk in iter(lambda: response.read(CHUNK_SIZE), b""):
                handle.write(chunk)
                digest.update(chunk)
                received += len(chunk)
    except (urllib.error.URLError, OSError) as error:
        temporary.unlink(missing_ok=True)
        raise ProvisionError(f"download of {url} failed: {error}") from error
    if received != int(expected_size):
        temporary.unlink(missing_ok=True)
        raise ProvisionError(
            f"{target.name}: expected {expected_size} bytes, received {received}"
        )
    if digest.hexdigest() != expected_sha256:
        temporary.unlink(missing_ok=True)
        raise ProvisionError(f"{target}: sha256 mismatch after download")
    temporary.replace(target)


def store_is_current(config: dict, destination: Path) -> bool:
    files = pinned_files(config)
    for name, entry in files.items():
        target = destination / name
        if not target.is_file():
            return False
        if sha256_of(target) != str(entry["sha256"]):
            return False
    return True


def provision_download(config: dict, destination: Path, allow: bool) -> str:
    files = pinned_files(config)
    if not files:
        raise ProvisionError(
            "voice/supertone/ carries no weights and voice/supertone-model.yaml has "
            "no download section; nothing to place"
        )
    if store_is_current(config, destination):
        log(f"{destination} already holds the pinned supertonic-3 files")
        return "already_current"
    if not allow:
        raise ProvisionError(
            f"supertonic-3 files are missing in {destination} and downloading is disabled "
            "(PIB_SUPERTONE_DOWNLOAD=0)"
        )
    destination.mkdir(parents=True, exist_ok=True)
    for name, entry in files.items():
        target = destination / name
        if target.is_file() and sha256_of(target) == str(entry["sha256"]):
            continue
        url = download_url(config, name)
        log(f"downloading {name} ({entry['size']} bytes) from {url}")
        fetch(url, target, str(entry["sha256"]), int(entry["size"]))
    log(f"placed pinned supertonic-3 files into {destination}")
    return "placed"


def provision(repo_root: Path, destination: Path, allow_download: bool) -> str:
    config = load_config(repo_root)
    destination.mkdir(parents=True, exist_ok=True)
    result = provision_vendored(repo_root / VENDORED_RELATIVE, destination)
    if result is not None:
        return result
    return provision_download(config, destination, allow_download)


def parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--repo-root",
        required=True,
        type=Path,
        help="checkout holding voice/supertone-model.yaml and voice/supertone/",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=Path(os.environ.get("SUPERTONE_MODEL_PATH", str(DEFAULT_DESTINATION))),
        help="voice model store (default: SUPERTONE_MODEL_PATH or %(default)s)",
    )
    parser.add_argument(
        "--no-download",
        action="store_true",
        help="place vendored weights only; never fetch the pinned files",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    allow_download = not args.no_download and os.environ.get(
        "PIB_SUPERTONE_DOWNLOAD", "1"
    ) not in ("0", "false", "no")
    try:
        result = provision(args.repo_root, args.destination, allow_download)
    except ProvisionError as error:
        log(str(error))
        print("result=failed", flush=True)
        return 1
    except Exception as error:  # noqa: BLE001
        log(f"unexpected {type(error).__name__}: {error}")
        print("result=failed", flush=True)
        return 1
    print(f"result={result}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
