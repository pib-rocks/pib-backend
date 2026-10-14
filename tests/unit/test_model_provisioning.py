"""Offline integration tests for the curated model provisioning CLI."""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_SCRIPT = REPO_ROOT / "setup" / "setup-pib.sh"


def _fixture_repo(tmp_path, include_blob=True):
    root = tmp_path / "repo"
    setup_dir = root / "setup"
    models_dir = root / "models"
    setup_dir.mkdir(parents=True)
    models_dir.mkdir()
    shutil.copy2(SETUP_SCRIPT, setup_dir / "setup-pib.sh")
    # setup-pib.sh sources the variant resolver, so the fixture repo needs it too
    installation_scripts_dir = setup_dir / "installation_scripts"
    installation_scripts_dir.mkdir()
    shutil.copy2(
        SETUP_SCRIPT.parent / "installation_scripts" / "resolve_hardware_variant.sh",
        installation_scripts_dir / "resolve_hardware_variant.sh",
    )

    content = b"fixture model blob\n"
    digest = hashlib.sha256(content).hexdigest()
    relative_file = "demo/demo.blob"
    if include_blob:
        blob = models_dir / relative_file
        blob.parent.mkdir()
        blob.write_bytes(content)
    (models_dir / "manifest.yaml").write_text(
        "\n".join(
            [
                "models:",
                "- model_id: demo",
                f"  file: {relative_file}",
                f"  sha256: {digest}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return root, relative_file


def _pack_asset(tmp_path, root):
    archive = tmp_path / "models-fixture.tar.gz"
    subprocess.run(
        [
            "tar",
            "-C",
            str(root / "models"),
            "-czf",
            str(archive),
            "manifest.yaml",
            "demo/demo.blob",
        ],
        check=True,
    )
    return archive, hashlib.sha256(archive.read_bytes()).hexdigest()


def _run(script, store, option, *, asset_url, asset_sha, cache):
    env = os.environ.copy()
    env["PIB_MODEL_STORE"] = str(store)
    env["PIB_MODEL_CACHE"] = str(cache)
    env["PIB_MODEL_ASSET_URL"] = asset_url
    env["PIB_MODEL_ASSET_SHA256"] = asset_sha
    env["PIB_WHISPER_DOWNLOAD"] = "0"
    return subprocess.run(
        ["bash", str(script), option],
        text=True,
        capture_output=True,
        env=env,
        check=False,
    )


def test_provision_places_manifest_and_is_idempotent(tmp_path):
    root, relative_file = _fixture_repo(tmp_path)
    script = root / "setup" / "setup-pib.sh"
    store = tmp_path / "store"
    cache = tmp_path / "cache"
    archive, digest = _pack_asset(tmp_path, root)

    first = _run(
        script,
        store,
        "--models",
        asset_url=archive.as_uri(),
        asset_sha=digest,
        cache=cache,
    )
    # The cache already matches, so a dead URL must not be contacted.
    second = _run(
        script,
        store,
        "--models",
        asset_url="http://127.0.0.1:9/missing-models.tar.gz",
        asset_sha="0" * 64,
        cache=cache,
    )

    assert first.returncode == 0, first.stdout + first.stderr
    assert second.returncode == 0, second.stdout + second.stderr
    assert (store / relative_file).read_bytes() == b"fixture model blob\n"
    assert (store / "manifest.yaml").read_text(encoding="utf-8") == (
        root / "models" / "manifest.yaml"
    ).read_text(encoding="utf-8")
    assert "demo: placed" in first.stdout
    assert "manifest.yaml: placed" in first.stdout
    assert "sha256 verified" in first.stdout
    assert "demo: already current" in second.stdout
    assert "manifest.yaml: already current" in second.stdout
    assert "cache already matches" in second.stdout


def test_verify_fails_with_actionable_message_for_corrupted_file(tmp_path):
    root, relative_file = _fixture_repo(tmp_path)
    script = root / "setup" / "setup-pib.sh"
    store = tmp_path / "store"
    cache = tmp_path / "cache"
    archive, digest = _pack_asset(tmp_path, root)
    assert (
        _run(
            script,
            store,
            "--models",
            asset_url=archive.as_uri(),
            asset_sha=digest,
            cache=cache,
        ).returncode
        == 0
    )
    (store / relative_file).write_bytes(b"corrupted\n")

    result = _run(
        script,
        store,
        "--verify-models",
        asset_url="http://127.0.0.1:9/missing-models.tar.gz",
        asset_sha="0" * 64,
        cache=cache,
    )

    assert result.returncode != 0
    assert "demo: sha256 mismatch in store" in result.stdout
    assert (
        "'./setup/setup-pib.sh --models' before starting Docker containers"
        in result.stdout
    )


def test_verify_is_offline_against_a_populated_store(tmp_path):
    root, _relative_file = _fixture_repo(tmp_path)
    script = root / "setup" / "setup-pib.sh"
    store = tmp_path / "store"
    cache = tmp_path / "cache"
    archive, digest = _pack_asset(tmp_path, root)
    assert (
        _run(
            script,
            store,
            "--models",
            asset_url=archive.as_uri(),
            asset_sha=digest,
            cache=cache,
        ).returncode
        == 0
    )

    result = _run(
        script,
        store,
        "--verify-models",
        asset_url="http://127.0.0.1:9/missing-models.tar.gz",
        asset_sha="0" * 64,
        cache=tmp_path / "unused-cache",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "demo: already current" in result.stdout
    assert "Fetching OAK model asset" not in result.stdout


def test_provision_returns_nonzero_when_asset_cannot_be_fetched(tmp_path):
    root, _ = _fixture_repo(tmp_path, include_blob=False)
    script = root / "setup" / "setup-pib.sh"
    store = tmp_path / "store"

    result = _run(
        script,
        store,
        "--models",
        asset_url="file:///no/such/oak-model-asset.tar.gz",
        asset_sha="0" * 64,
        cache=tmp_path / "cache",
    )

    assert result.returncode != 0
    assert "OAK model asset could not be fetched and the model store" in result.stdout
    assert f"at {store} is empty" in result.stdout
    assert "PIB_MODEL_ASSET_URL" in result.stdout
    assert "Model provisioning failed" in result.stdout
    assert not (store / "demo" / "demo.blob").exists()
    assert not (store / "manifest.yaml").exists()
