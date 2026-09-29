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


def _run(script, store, option):
    env = os.environ.copy()
    env["PIB_MODEL_STORE"] = str(store)
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

    first = _run(script, store, "--models")
    second = _run(script, store, "--models")

    assert first.returncode == 0, first.stdout + first.stderr
    assert second.returncode == 0, second.stdout + second.stderr
    assert (store / relative_file).read_bytes() == b"fixture model blob\n"
    assert (store / "manifest.yaml").read_text(encoding="utf-8") == (
        root / "models" / "manifest.yaml"
    ).read_text(encoding="utf-8")
    assert "demo: placed" in first.stdout
    assert "manifest.yaml: placed" in first.stdout
    assert "demo: already current" in second.stdout
    assert "manifest.yaml: already current" in second.stdout


def test_verify_fails_with_actionable_message_for_corrupted_file(tmp_path):
    root, relative_file = _fixture_repo(tmp_path)
    script = root / "setup" / "setup-pib.sh"
    store = tmp_path / "store"
    assert _run(script, store, "--models").returncode == 0
    (store / relative_file).write_bytes(b"corrupted\n")

    result = _run(script, store, "--verify-models")

    assert result.returncode != 0
    assert "demo: sha256 mismatch in store" in result.stdout
    assert (
        "'./setup/setup-pib.sh --models' before starting Docker containers"
        in result.stdout
    )


def test_provision_returns_nonzero_when_vendored_blob_is_missing(tmp_path):
    root, _ = _fixture_repo(tmp_path, include_blob=False)
    script = root / "setup" / "setup-pib.sh"

    result = _run(script, tmp_path / "store", "--models")

    assert result.returncode != 0
    assert "demo: vendored file missing" in result.stdout
    assert "Model provisioning failed" in result.stdout
