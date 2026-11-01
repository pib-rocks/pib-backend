"""Unit tests for GET /robot-model/* and tools/install_robot_model.py."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

from service import robot_model_service as rms

REPO_ROOT = Path(__file__).resolve().parents[2]
INSTALLER = REPO_ROOT / "tools" / "install_robot_model.py"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_fixture_tree(root: Path, version: str = "v0.1.1-test") -> None:
    (root / "meshes").mkdir(parents=True)
    (root / "meshes" / "root.glb").write_bytes(b"glb-fixture")
    (root / "meshes" / "link_a.glb").write_bytes(b"second")
    model_path = root / "model.json"
    model_path.write_text(
        json.dumps(
            {
                "name": "pib",
                "build": "abc123build",
                "links": [{"name": "root"}, {"name": "link_a"}],
                "joints": [{"name": "j1"}, {"name": "j2"}, {"name": "j3"}],
            }
        ),
        encoding="utf-8",
    )
    mapping_path = root / "mapping.json"
    mapping_path.write_text(
        json.dumps(
            {
                "joints": [
                    {"urdfJoint": "j2", "class": "driven", "motor": "motor_a"},
                    {"urdfJoint": "j1", "class": "coupled", "motor": None},
                ]
            }
        ),
        encoding="utf-8",
    )
    (root / "version.json").write_text(
        json.dumps(
            {
                "version": version,
                "sha256": {
                    "model.json": _sha256(model_path),
                    "mapping.json": _sha256(mapping_path),
                },
            }
        ),
        encoding="utf-8",
    )


@pytest.fixture()
def robot_model_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    model_dir = tmp_path / "robot_model"
    _write_fixture_tree(model_dir)
    monkeypatch.setenv("ROBOT_MODEL_DIR", str(model_dir))
    rms.clear_manifest_cache()
    yield model_dir
    rms.clear_manifest_cache()


@pytest.fixture()
def empty_robot_model_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    model_dir = tmp_path / "empty_model"
    model_dir.mkdir()
    monkeypatch.setenv("ROBOT_MODEL_DIR", str(model_dir))
    rms.clear_manifest_cache()
    yield model_dir
    rms.clear_manifest_cache()


def test_manifest_shape(app, robot_model_dir):
    with app.test_client() as client:
        response = client.get("/robot-model/manifest.json")

    assert response.status_code == 200
    assert response.content_type.startswith("application/json")
    body = response.get_json()
    assert body["version"] == "v0.1.1-test"
    assert body["modelBuild"] == "abc123build"
    assert body["links"] == 2
    assert body["joints"] == 2
    assert body["files"] == {
        "model": "/robot-model/model.json",
        "mapping": "/robot-model/mapping.json",
    }
    names = {entry["name"] for entry in body["meshes"]}
    assert names == {"link_a.glb", "root.glb"}
    for entry in body["meshes"]:
        assert entry["url"] == f"/robot-model/meshes/{entry['name']}"
        assert entry["size"] > 0


def test_manifest_not_installed(app, empty_robot_model_dir):
    with app.test_client() as client:
        response = client.get("/robot-model/manifest.json")

    assert response.status_code == 404
    body = response.get_json()
    assert body["error"] == "no robot model installed"
    assert "install_robot_model.py" in body["hint"]


def test_cache_control_immutable_with_version_query(app, robot_model_dir):
    version = "v0.1.1-test"
    with app.test_client() as client:
        with_v = client.get(f"/robot-model/meshes/root.glb?v={version}")
        without_v = client.get("/robot-model/meshes/root.glb")

    assert with_v.status_code == 200
    assert without_v.status_code == 200
    assert with_v.headers.get("Cache-Control") == "public, max-age=31536000, immutable"
    assert without_v.headers.get("Cache-Control") == "no-cache"


def test_etag_returns_304(app, robot_model_dir):
    with app.test_client() as client:
        first = client.get("/robot-model/model.json")
        etag = first.headers.get("ETag")
        assert etag
        second = client.get("/robot-model/model.json", headers={"If-None-Match": etag})

    assert first.status_code == 200
    assert second.status_code == 304
    assert second.headers.get("ETag") == etag


@pytest.mark.parametrize(
    "mesh_name",
    [
        "../model.json",
        "/etc/passwd",
        "x.glb/../../etc/passwd",
        "..",
        "not-a-mesh.txt",
    ],
)
def test_mesh_traversal_rejected(app, robot_model_dir, mesh_name):
    with app.test_client() as client:
        response = client.get(f"/robot-model/meshes/{mesh_name}")

    assert response.status_code == 404


def test_installer_wrong_sha256_leaves_previous_install(tmp_path: Path):
    dest = tmp_path / "installed"
    _write_fixture_tree(dest, version="v-original")

    bundle = tmp_path / "bundle"
    _write_fixture_tree(bundle, version="v-new")
    zip_path = tmp_path / "bundle.zip"
    with zipfile.ZipFile(zip_path, "w") as zf:
        for path in bundle.rglob("*"):
            if path.is_file():
                zf.write(path, path.relative_to(bundle).as_posix())

    good_digest = __import__("hashlib").sha256(zip_path.read_bytes()).hexdigest()
    wrong_digest = "0" * 64

    result = subprocess.run(
        [
            sys.executable,
            str(INSTALLER),
            "--from",
            str(zip_path),
            "--dest",
            str(dest),
            "--sha256",
            wrong_digest,
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "sha256 mismatch" in result.stderr

    assert (
        json.loads((dest / "version.json").read_text(encoding="utf-8"))["version"]
        == "v-original"
    )

    ok = subprocess.run(
        [
            sys.executable,
            str(INSTALLER),
            "--from",
            str(zip_path),
            "--dest",
            str(dest),
            "--sha256",
            good_digest,
        ],
        capture_output=True,
        text=True,
    )
    assert ok.returncode == 0, ok.stderr
    assert (
        json.loads((dest / "version.json").read_text(encoding="utf-8"))["version"]
        == "v-new"
    )


def test_manifest_lists_joint_and_motor_sets(app, robot_model_dir):
    with app.test_client() as client:
        body = client.get("/robot-model/manifest.json").get_json()

    assert body["jointNames"] == ["j2", "j1"]
    assert body["motors"] == ["motor_a"]
    assert body["mappingSha256"] == _sha256(robot_model_dir / "mapping.json")
    assert body["integrity"] == "ok"


def test_manifest_reports_integrity_mismatch(app, robot_model_dir):
    # A half-updated directory: the mapping no longer matches the hash recorded at install time.
    (robot_model_dir / "mapping.json").write_text("{}", encoding="utf-8")
    rms.clear_manifest_cache()

    with app.test_client() as client:
        body = client.get("/robot-model/manifest.json").get_json()

    assert body["integrity"] == "mismatch"
