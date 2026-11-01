"""Serve the installed pib-3d web asset bundle from ROBOT_MODEL_DIR."""

from __future__ import annotations

import hashlib
import json
import os
import re
from typing import Any

from config import Config

_MESH_NAME = re.compile(r"^[A-Za-z0-9._-]+\.glb$")

_manifest_cache: dict[tuple[float, float, float], dict[str, Any]] = {}


def robot_model_dir() -> str:
    return os.getenv("ROBOT_MODEL_DIR", Config.ROBOT_MODEL_DIR)


def clear_manifest_cache() -> None:
    _manifest_cache.clear()


def _required_paths(base: str) -> tuple[str, str, str, str]:
    return (
        os.path.join(base, "version.json"),
        os.path.join(base, "model.json"),
        os.path.join(base, "mapping.json"),
        os.path.join(base, "meshes"),
    )


def is_installed(base: str | None = None) -> bool:
    root = base if base is not None else robot_model_dir()
    version_path, model_path, mapping_path, meshes_dir = _required_paths(root)
    return (
        os.path.isfile(version_path)
        and os.path.isfile(model_path)
        and os.path.isfile(mapping_path)
        and os.path.isdir(meshes_dir)
    )


def installed_version(base: str | None = None) -> str | None:
    root = base if base is not None else robot_model_dir()
    version_path = os.path.join(root, "version.json")
    if not os.path.isfile(version_path):
        return None
    with open(version_path, encoding="utf-8") as fh:
        data = json.load(fh)
    version = data.get("version")
    return str(version) if version is not None else None


def load_manifest(base: str | None = None) -> dict[str, Any] | None:
    root = base if base is not None else robot_model_dir()
    if not is_installed(root):
        return None

    version_path, model_path, mapping_path, meshes_dir = _required_paths(root)
    cache_key = (
        os.path.getmtime(version_path),
        os.path.getmtime(model_path),
        os.path.getmtime(mapping_path),
    )
    cached = _manifest_cache.get(cache_key)
    if cached is not None:
        return cached

    with open(version_path, encoding="utf-8") as fh:
        version_doc = json.load(fh)
    with open(model_path, encoding="utf-8") as fh:
        model_doc = json.load(fh)
    with open(mapping_path, encoding="utf-8") as fh:
        mapping_doc = json.load(fh)

    version = str(version_doc.get("version", ""))
    model_build = str(model_doc.get("build", ""))
    links = model_doc.get("links")
    if isinstance(links, list):
        link_count = len(links)
    else:
        stats = model_doc.get("stats") or {}
        link_count = int(stats.get("links", 0))

    joint_entries = mapping_doc.get("joints")
    if not isinstance(joint_entries, list):
        joint_entries = model_doc.get("joints") or []
    joint_count = len(joint_entries)
    joint_names = [
        str(e.get("urdfJoint", "")) for e in joint_entries if e.get("urdfJoint")
    ]
    motor_names = sorted(
        str(e["motor"])
        for e in joint_entries
        if e.get("class") == "driven" and e.get("motor")
    )

    mesh_entries = []
    for name in sorted(os.listdir(meshes_dir)):
        if not name.endswith(".glb"):
            continue
        path = os.path.join(meshes_dir, name)
        if not os.path.isfile(path):
            continue
        mesh_entries.append(
            {
                "name": name,
                "size": os.path.getsize(path),
                "url": f"/robot-model/meshes/{name}",
            }
        )

    manifest: dict[str, Any] = {
        "version": version,
        "modelBuild": model_build,
        "links": link_count,
        "joints": joint_count,
        # The sets, not just the counts: a viewer must be able to see whether the model it
        # loaded describes the robot in front of it (PR-2151).
        "jointNames": joint_names,
        "motors": motor_names,
        "mappingSha256": (version_doc.get("sha256") or {}).get("mapping.json"),
        "integrity": _integrity(version_doc, model_path, mapping_path),
        "files": {
            "model": "/robot-model/model.json",
            "mapping": "/robot-model/mapping.json",
        },
        "meshes": mesh_entries,
    }
    _manifest_cache.clear()
    _manifest_cache[cache_key] = manifest
    return manifest


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _integrity(version_doc: dict[str, Any], model_path: str, mapping_path: str) -> str:
    """Compare the files against the hashes recorded at install time.

    A half-updated directory (old model, new mapping) would otherwise be served happily and the
    viewer would show a robot that does not exist.
    """
    expected = version_doc.get("sha256") or {}
    checks = (
        ("model.json", model_path),
        ("mapping.json", mapping_path),
    )
    for name, path in checks:
        want = expected.get(name)
        if want and _sha256(path) != want:
            return "mismatch"
    return "ok"


def file_path(relative: str, base: str | None = None) -> str | None:
    root = base if base is not None else robot_model_dir()
    if not is_installed(root):
        return None
    path = os.path.join(root, relative)
    if not os.path.isfile(path):
        return None
    return path


def resolve_mesh(name: str, base: str | None = None) -> str | None:
    if not _MESH_NAME.fullmatch(name):
        return None
    root = base if base is not None else robot_model_dir()
    if not is_installed(root):
        return None
    meshes_dir = os.path.join(root, "meshes")
    candidate = os.path.join(meshes_dir, name)
    if not os.path.isfile(candidate):
        return None
    resolved_root = os.path.realpath(root)
    resolved_path = os.path.realpath(candidate)
    if not resolved_path.startswith(resolved_root + os.sep):
        return None
    return resolved_path


def file_etag(path: str) -> str:
    stat = os.stat(path)
    return f'"{int(stat.st_mtime)}-{stat.st_size}"'
