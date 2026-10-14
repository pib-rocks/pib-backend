#!/usr/bin/env python3
"""Place the faster-whisper weights into the voice model store at setup time.

Self-contained data step for setup-pib.sh: standard library plus PyYAML (the
``python3-yaml`` package that install_system_packages puts on the host). It
does not import the voice_assistant ROS package, so it runs before any
workspace is built and independently of where setup-pib.sh was downloaded to.

Sources for the weights, in this order:

1. the store itself - ``<destination>/<size>/model.bin`` whose sha256 matches
   the pinned digest (or the vendored copy) is left alone;
2. the vendored tree ``<repo>/voice/whisper/`` (flat, or ``<size>/`` dirs);
3. the pinned files listed under ``download:`` in ``voice/whisper-model.yaml``,
   fetched once while the network is up so the first start works offline.

The engine inside the container never downloads (``download_at_first_use``
stays false); this step is the only place weights enter the robot.

The last stdout line is ``result=<word>`` with one of ``placed``,
``already_current`` or ``failed``; everything else goes to stderr so the
setup log carries the real reason when something goes wrong.
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

WEIGHTS_NAME = "model.bin"
DEFAULT_DESTINATION = Path("/data/voice/models/whisper")
CONFIG_RELATIVE = Path("voice") / "whisper-model.yaml"
VENDORED_RELATIVE = Path("voice") / "whisper"
DOWNLOAD_TIMEOUT_SECONDS = 60
CHUNK_SIZE = 1024 * 1024
USER_AGENT = "pib-setup-whisper-provision/1.0"


class ProvisionError(Exception):
    """A reason the weights could not be placed; printed verbatim to stderr."""


def log(message: str) -> None:
    print(f"whisper model: {message}", file=sys.stderr, flush=True)


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
        import yaml  # python3-yaml; installed by install_system_packages
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
    """``files`` from the ``download`` section, validated for sha256 and size."""
    download = config.get("download")
    if not isinstance(download, dict):
        return {}
    files = download.get("files")
    if not isinstance(files, dict) or not files:
        raise ProvisionError("download.files in voice/whisper-model.yaml is empty")
    for name, entry in files.items():
        if not isinstance(entry, dict) or "sha256" not in entry or "size" not in entry:
            raise ProvisionError(f"download.files.{name} needs sha256 and size")
    if WEIGHTS_NAME not in files:
        raise ProvisionError(f"download.files does not list {WEIGHTS_NAME}")
    return files


def download_url(config: dict, name: str) -> str:
    download = config["download"]
    repository = str(download.get("repository", "")).rstrip("/")
    revision = str(download.get("revision", "main"))
    if not repository:
        raise ProvisionError("download.repository is not set")
    return f"{repository}/resolve/{revision}/{name}"


def _copy_tree(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for item in source.iterdir():
        if not item.is_file():
            continue
        temporary = destination / f".{item.name}.tmp"
        shutil.copy2(item, temporary)
        temporary.replace(destination / item.name)


def _place_vendored(source: Path, destination: Path) -> str:
    weights = source / WEIGHTS_NAME
    current = destination / WEIGHTS_NAME
    if current.is_file() and sha256_of(current) == sha256_of(weights):
        log(f"{destination} already holds the vendored weights")
        return "already_current"
    _copy_tree(source, destination)
    log(f"copied vendored weights from {source} to {destination}")
    return "placed"


def provision_vendored(vendored: Path, destination: Path) -> Optional[str]:
    """Copy voice/whisper/ when it carries weights; None when it does not."""
    if (vendored / WEIGHTS_NAME).is_file():
        return _place_vendored(vendored, destination)
    if not vendored.is_dir():
        return None
    results = []
    for child in sorted(vendored.iterdir()):
        if child.is_dir() and (child / WEIGHTS_NAME).is_file():
            results.append(_place_vendored(child, destination / child.name))
    if not results:
        return None
    return "placed" if "placed" in results else "already_current"


def fetch(url: str, target: Path, expected_sha256: str, expected_size: int) -> None:
    """Stream ``url`` into ``target`` and verify size and sha256 on the way."""
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
        raise ProvisionError(f"{target.name}: sha256 mismatch after download")
    temporary.replace(target)


def provision_download(config: dict, destination: Path, allow: bool) -> str:
    files = pinned_files(config)
    if not files:
        raise ProvisionError(
            "voice/whisper/ carries no weights and voice/whisper-model.yaml has "
            "no download section; nothing to place"
        )
    size_dir = destination / str(config.get("model_size", "small"))
    weights = size_dir / WEIGHTS_NAME
    pinned = files[WEIGHTS_NAME]
    if weights.is_file() and sha256_of(weights) == pinned["sha256"]:
        complete = all((size_dir / name).is_file() for name in files)
        if complete:
            log(f"{size_dir} already holds the pinned weights")
            return "already_current"
        log(f"{size_dir} holds the weights but lacks companion files; completing")
    elif weights.is_file():
        log(f"{weights} does not match the pinned sha256; replacing")
    if not allow:
        raise ProvisionError(
            f"weights are missing in {size_dir} and downloading is disabled "
            "(PIB_WHISPER_DOWNLOAD=0)"
        )
    for name, entry in files.items():
        target = size_dir / name
        if target.is_file() and sha256_of(target) == entry["sha256"]:
            continue
        url = download_url(config, name)
        log(f"downloading {name} ({entry['size']} bytes) from {url}")
        fetch(url, target, str(entry["sha256"]), int(entry["size"]))
    log(f"placed pinned weights into {size_dir}")
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
        help="checkout holding voice/whisper-model.yaml and voice/whisper/",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=Path(os.environ.get("WHISPER_MODEL_PATH", str(DEFAULT_DESTINATION))),
        help="voice model store (default: WHISPER_MODEL_PATH or %(default)s)",
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
        "PIB_WHISPER_DOWNLOAD", "1"
    ) not in ("0", "false", "no")
    try:
        result = provision(args.repo_root, args.destination, allow_download)
    except ProvisionError as error:
        log(str(error))
        print("result=failed", flush=True)
        return 1
    except Exception as error:  # noqa: BLE001 - the log must carry the reason
        log(f"unexpected {type(error).__name__}: {error}")
        print("result=failed", flush=True)
        return 1
    print(f"result={result}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
