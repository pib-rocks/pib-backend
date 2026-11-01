#!/usr/bin/env python3
"""Download or unpack a pib-3d web asset zip into ROBOT_MODEL_DIR."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile


def _default_dest() -> str:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    base = os.path.join(repo_root, "pib_api", "flask", "robot_model")
    return os.getenv("ROBOT_MODEL_DIR", base)


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _content_root(extracted: str) -> str:
    entries = [e for e in os.listdir(extracted) if not e.startswith(".")]
    if len(entries) == 1:
        only = os.path.join(extracted, entries[0])
        if os.path.isdir(only):
            return only
    return extracted


def _validate_tree(root: str) -> tuple[int, str]:
    version_path = os.path.join(root, "version.json")
    model_path = os.path.join(root, "model.json")
    mapping_path = os.path.join(root, "mapping.json")
    meshes_dir = os.path.join(root, "meshes")
    missing = [
        name
        for name, path in (
            ("version.json", version_path),
            ("model.json", model_path),
            ("mapping.json", mapping_path),
        )
        if not os.path.isfile(path)
    ]
    if not os.path.isdir(meshes_dir):
        missing.append("meshes/")
    if missing:
        raise SystemExit(f"invalid bundle: missing {', '.join(missing)}")

    with open(version_path, encoding="utf-8") as fh:
        version_doc = json.load(fh)
    version = version_doc.get("version")
    if not version:
        raise SystemExit("invalid bundle: version.json has no version field")

    mesh_count = sum(
        1
        for name in os.listdir(meshes_dir)
        if name.endswith(".glb") and os.path.isfile(os.path.join(meshes_dir, name))
    )
    if mesh_count == 0:
        raise SystemExit("invalid bundle: meshes/ contains no .glb files")
    return mesh_count, str(version)


def _atomic_replace(dest: str, bundle_root: str) -> None:
    dest = os.path.abspath(dest)
    bundle_root = os.path.abspath(bundle_root)
    parent = os.path.dirname(dest)
    os.makedirs(parent, exist_ok=True)
    backup = dest + ".previous"
    if os.path.exists(backup):
        shutil.rmtree(backup)
    if os.path.exists(dest):
        os.rename(dest, backup)
    try:
        os.rename(bundle_root, dest)
    except OSError:
        if not os.path.exists(dest) and os.path.exists(backup):
            os.rename(backup, dest)
        raise
    if os.path.exists(backup):
        shutil.rmtree(backup)


def install_from_zip(zip_path: str, dest: str, expected_sha256: str | None) -> None:
    zip_path = os.path.abspath(zip_path)
    if not os.path.isfile(zip_path):
        raise SystemExit(f"zip not found: {zip_path}")

    data = open(zip_path, "rb").read()
    digest = _sha256_bytes(data)
    if expected_sha256 and digest.lower() != expected_sha256.lower():
        raise SystemExit(
            f"sha256 mismatch: expected {expected_sha256.lower()}, got {digest}"
        )

    dest = os.path.abspath(dest)
    dest_parent = os.path.dirname(dest) or "."
    staging = tempfile.mkdtemp(prefix=".robot_model_staging_", dir=dest_parent)
    try:
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(staging)
        bundle_root = _content_root(staging)
        mesh_count, version = _validate_tree(bundle_root)
        _atomic_replace(dest, bundle_root)
        if bundle_root != staging and os.path.exists(staging):
            shutil.rmtree(staging, ignore_errors=True)
        print(f"installed robot model {version} ({mesh_count} meshes) -> {dest}")
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def download_to_temp(url: str) -> str:
    fd, path = tempfile.mkstemp(prefix="robot_model_", suffix=".zip")
    os.close(fd)
    try:
        with urllib.request.urlopen(url) as response, open(path, "wb") as out:
            shutil.copyfileobj(response, out)
    except Exception:
        os.remove(path)
        raise
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--url", help="HTTPS URL of the asset zip")
    source.add_argument("--from", dest="from_zip", help="Local zip path")
    parser.add_argument("--sha256", help="Expected sha256 of the zip file")
    parser.add_argument(
        "--dest",
        default=None,
        help=f"Install directory (default: {_default_dest()})",
    )
    args = parser.parse_args(argv)
    dest = args.dest or _default_dest()

    temp_zip: str | None = None
    try:
        if args.url:
            temp_zip = download_to_temp(args.url)
            zip_path = temp_zip
        else:
            zip_path = args.from_zip
        install_from_zip(zip_path, dest, args.sha256)
    except SystemExit as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        if temp_zip and os.path.exists(temp_zip):
            os.remove(temp_zip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
