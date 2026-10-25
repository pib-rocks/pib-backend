"""Supertone supertonic-3 weights land in the store at setup time (PR-2054).

installation_scripts/provision_supertone_model.py is exercised with urllib
replaced by an in-memory server, mirroring test_setup_whisper_provisioning.py.
"""

from __future__ import annotations

import hashlib
import importlib.util
import io
import os
import re
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_PIB = REPO_ROOT / "setup" / "setup-pib.sh"
HELPER = REPO_ROOT / "setup" / "installation_scripts" / "provision_supertone_model.py"
MODEL_CONFIG = REPO_ROOT / "voice" / "supertone-model.yaml"

MARKER = "onnx/tts.json"
FILES = {
    MARKER: b'{"sample_rate": 44100}',
    "onnx/unicode_indexer.json": b"[]",
    "voice_styles/M1.json": b'{"style": "m1"}',
}


def _load_helper():
    spec = importlib.util.spec_from_file_location("provision_supertone_model", HELPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_config(
    repo_root: Path, files=FILES, repository="https://models.test/supertone"
):
    (repo_root / "voice" / "supertone").mkdir(parents=True, exist_ok=True)
    document = {
        "path": "/data/voice/models/supertone/",
        "model": "supertonic-3",
        "download_at_first_use": False,
        "download": {
            "repository": repository,
            "revision": "abc123",
            "files": {
                name: {"sha256": hashlib.sha256(body).hexdigest(), "size": len(body)}
                for name, body in files.items()
            },
        },
    }
    (repo_root / "voice" / "supertone-model.yaml").write_text(
        yaml.safe_dump(document), encoding="utf-8"
    )
    return document


class _FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def _serve(monkeypatch, module, bodies, requested):
    def fake_urlopen(request, timeout):
        url = request.full_url
        requested.append(url)
        name = url.split("/resolve/abc123/", 1)[-1]
        if name not in bodies:
            raise module.urllib.error.URLError(f"404 {name}")
        return _FakeResponse(bodies[name])

    monkeypatch.setattr(module.urllib.request, "urlopen", fake_urlopen)


def test_pinned_files_are_downloaded_once_and_verified(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    requested = []
    _serve(monkeypatch, module, FILES, requested)
    store = tmp_path / "store"

    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    captured = capsys.readouterr()
    assert captured.out.strip().splitlines()[-1] == "result=placed"
    for name, body in FILES.items():
        assert (store / name).read_bytes() == body
    assert sorted(requested) == sorted(
        f"https://models.test/supertone/resolve/abc123/{name}" for name in FILES
    )

    requested.clear()
    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "result=already_current"
    assert requested == []


def test_vendored_tree_wins_without_download(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    vendored = repo / "voice" / "supertone" / "onnx"
    vendored.mkdir(parents=True)
    (vendored / "tts.json").write_bytes(b"operator tree")
    requested = []
    _serve(monkeypatch, module, FILES, requested)
    store = tmp_path / "store"

    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "result=placed"
    assert (store / "onnx" / "tts.json").read_bytes() == b"operator tree"
    assert requested == []


def test_downloads_can_be_switched_off(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    _serve(monkeypatch, module, FILES, [])
    monkeypatch.setenv("PIB_SUPERTONE_DOWNLOAD", "0")

    assert module.main(["--repo-root", str(repo), "--destination", str(tmp_path / "s")])
    captured = capsys.readouterr()
    assert "downloading is disabled (PIB_SUPERTONE_DOWNLOAD=0)" in captured.err
    assert captured.out.strip().splitlines()[-1] == "result=failed"


def test_the_shipped_configuration_pins_supertonic3_completely():
    document = yaml.safe_load(MODEL_CONFIG.read_text(encoding="utf-8"))
    download = document["download"]

    assert download["repository"] == "https://huggingface.co/Supertone/supertonic-3"
    assert re.fullmatch(r"[0-9a-f]{40}", download["revision"])
    assert MARKER in download["files"]
    for name, entry in download["files"].items():
        assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), name
        assert int(entry["size"]) > 0, name
    assert document["download_at_first_use"] is False


SETUP_PRELUDE = """
function print() { echo "[$1][[ ${2:-} ]]"; }
function command_exists() { command -v "$@" >/dev/null 2>&1; }
"""


def _extract_bash_function(script: Path, name: str) -> str:
    text = script.read_text(encoding="utf-8")
    match = re.search(
        rf"^(?:function )?{re.escape(name)}\(\) \{{\n.*?^\}}\n",
        text,
        re.DOTALL | re.MULTILINE,
    )
    assert match, f"function {name} not found in {script}"
    return match.group(0)


def _run_wrapper(tmp_path: Path, backend_dir: Path, store: Path, **env_overrides):
    script = (
        SETUP_PRELUDE
        + _extract_bash_function(SETUP_PIB, "ensure_model_store_directory")
        + _extract_bash_function(SETUP_PIB, "whisper_repo_root")
        + _extract_bash_function(SETUP_PIB, "provision_supertone_model")
        + "\nprovision_supertone_model\necho rc=$?\n"
    )
    env = dict(os.environ)
    env.update(
        BACKEND_DIR=str(backend_dir),
        SUPERTONE_MODEL_PATH=str(store),
        PIB_SUPERTONE_DOWNLOAD="0",
    )
    env.update({key: str(value) for key, value in env_overrides.items()})
    return subprocess.run(
        ["bash", "-c", script],
        capture_output=True,
        check=False,
        env=env,
        text=True,
        cwd=str(tmp_path),
    )


def test_wrapper_places_vendored_weights(tmp_path):
    backend = tmp_path / "pib-backend"
    (backend / "setup" / "installation_scripts").mkdir(parents=True)
    (
        backend / "setup" / "installation_scripts" / "provision_supertone_model.py"
    ).write_bytes(HELPER.read_bytes())
    _write_config(backend)
    vendored = backend / "voice" / "supertone" / "onnx"
    vendored.mkdir(parents=True)
    (vendored / "tts.json").write_bytes(b"vendored")
    store = tmp_path / "store"

    result = _run_wrapper(tmp_path, backend, store)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert (store / "onnx" / "tts.json").read_bytes() == b"vendored"
