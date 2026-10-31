"""Serve the installed pib-3d web asset bundle from ROBOT_MODEL_DIR."""

from __future__ import annotations

import json
import os
import re
from typing import Any

from config import Config

_MESH_NAME = re.compile(r"^[A-Za-z0-9._-]+\.glb$")

_manifest_cache: dict[tuple[float, float], dict[str, Any]] = {}


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
    cache_key = (os.path.getmtime(version_path), os.path.getmtime(model_path))
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

    joints = mapping_doc.get("joints")
    if isinstance(joints, list):
        joint_count = len(joints)
    else:
        joint_count = len(model_doc.get("joints") or [])

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
        "files": {
            "model": "/robot-model/model.json",
            "mapping": "/robot-model/mapping.json",
        },
        "meshes": mesh_entries,
    }
    _manifest_cache.clear()
    _manifest_cache[cache_key] = manifest
    return manifest


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
