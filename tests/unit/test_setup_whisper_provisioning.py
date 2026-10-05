"""The whisper weights land in the store at setup time, without the ROS package (PR-1910).

installation_scripts/provision_whisper_model.py is exercised directly with urllib
replaced by an in-memory server; the bash wrapper provision_whisper_model() is cut
out of setup-pib.sh and run against a fixture checkout.
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
HELPER = REPO_ROOT / "setup" / "installation_scripts" / "provision_whisper_model.py"
MODEL_CONFIG = REPO_ROOT / "voice" / "whisper-model.yaml"

FILES = {
    "model.bin": b"pinned small weights",
    "config.json": b"{}",
    "tokenizer.json": b'{"tokens": []}',
    "vocabulary.txt": b"<|startoftranscript|>\n",
}


def _load_helper():
    spec = importlib.util.spec_from_file_location("provision_whisper_model", HELPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_config(repo_root: Path, files=FILES, repository="https://models.test/x"):
    (repo_root / "voice" / "whisper").mkdir(parents=True, exist_ok=True)
    document = {
        "path": "/data/voice/models/whisper/",
        "model_size": "small",
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
    (repo_root / "voice" / "whisper-model.yaml").write_text(
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
        name = url.rsplit("/", 1)[1]
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
        assert (store / "small" / name).read_bytes() == body
    assert sorted(requested) == sorted(
        f"https://models.test/x/resolve/abc123/{name}" for name in FILES
    )
    assert not list((store / "small").glob(".*.tmp"))

    requested.clear()
    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "result=already_current"
    assert requested == []


def test_a_corrupt_download_is_rejected_with_the_reason_in_the_log(
    tmp_path, monkeypatch, capsys
):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    served = dict(FILES)
    served["model.bin"] = b"pinned small weightX"  # same length, other bytes
    _serve(monkeypatch, module, served, [])
    store = tmp_path / "store"

    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 1
    captured = capsys.readouterr()
    assert captured.out.strip().splitlines()[-1] == "result=failed"
    assert "model.bin: sha256 mismatch after download" in captured.err
    assert not (store / "small" / "model.bin").exists()
    assert not list((store / "small").glob(".*.tmp"))


def test_a_network_error_names_the_url_and_fails(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    _serve(monkeypatch, module, {}, [])

    assert module.main(["--repo-root", str(repo), "--destination", str(tmp_path / "s")])
    captured = capsys.readouterr()
    assert "download of https://models.test/x/resolve/abc123/" in captured.err
    assert "failed" in captured.err


def test_vendored_weights_win_and_nothing_is_downloaded(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    vendored = repo / "voice" / "whisper" / "small"
    vendored.mkdir(parents=True)
    (vendored / "model.bin").write_bytes(b"operator supplied weights")
    (vendored / "config.json").write_text("{}", encoding="utf-8")
    requested = []
    _serve(monkeypatch, module, FILES, requested)
    store = tmp_path / "store"

    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "result=placed"
    assert (store / "small" / "model.bin").read_bytes() == b"operator supplied weights"
    assert requested == []

    assert module.main(["--repo-root", str(repo), "--destination", str(store)]) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "result=already_current"


def test_downloads_can_be_switched_off_and_say_so(tmp_path, monkeypatch, capsys):
    module = _load_helper()
    repo = tmp_path / "repo"
    _write_config(repo)
    _serve(monkeypatch, module, FILES, [])
    monkeypatch.setenv("PIB_WHISPER_DOWNLOAD", "0")

    assert module.main(["--repo-root", str(repo), "--destination", str(tmp_path / "s")])
    captured = capsys.readouterr()
    assert "downloading is disabled (PIB_WHISPER_DOWNLOAD=0)" in captured.err
    assert captured.out.strip().splitlines()[-1] == "result=failed"


def test_a_missing_configuration_is_a_named_failure(tmp_path, capsys):
    module = _load_helper()

    assert module.main(["--repo-root", str(tmp_path), "--destination", str(tmp_path)])
    captured = capsys.readouterr()
    assert "configuration not found at" in captured.err
    assert captured.out.strip().splitlines()[-1] == "result=failed"


def test_the_shipped_configuration_pins_the_small_model_completely():
    document = yaml.safe_load(MODEL_CONFIG.read_text(encoding="utf-8"))
    download = document["download"]

    assert download["repository"].startswith("https://huggingface.co/")
    assert re.fullmatch(r"[0-9a-f]{40}", download["revision"])
    assert set(download["files"]) >= {"model.bin", "config.json", "tokenizer.json"}
    for name, entry in download["files"].items():
        assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), name
        assert int(entry["size"]) > 0, name
    assert int(download["files"]["model.bin"]["size"]) > 100_000_000
    # The engine still never downloads; only setup does, and only these files.
    assert document["download_at_first_use"] is False


def test_the_helper_does_not_import_the_ros_package():
    text = HELPER.read_text(encoding="utf-8")
    imports = re.findall(r"^\s*(?:from|import)\s+(\S+)", text, re.MULTILINE)
    assert imports, "no imports found"
    assert not [name for name in imports if name.startswith("voice_assistant")]
    assert not [name for name in imports if name.startswith("rclpy")]


# ---- the bash wrapper -------------------------------------------------------------------

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
        + _extract_bash_function(SETUP_PIB, "provision_whisper_model")
        + "\nprovision_whisper_model\necho rc=$?\n"
    )
    env = dict(os.environ)
    env.update(
        BACKEND_DIR=str(backend_dir),
        WHISPER_MODEL_PATH=str(store),
        PIB_WHISPER_DOWNLOAD="0",
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


def _fixture_checkout(tmp_path: Path, with_weights: bool) -> Path:
    backend = tmp_path / "pib-backend"
    (backend / "setup" / "installation_scripts").mkdir(parents=True)
    (
        backend / "setup" / "installation_scripts" / "provision_whisper_model.py"
    ).write_bytes(HELPER.read_bytes())
    _write_config(backend)
    if with_weights:
        vendored = backend / "voice" / "whisper" / "small"
        vendored.mkdir(parents=True)
        (vendored / "model.bin").write_bytes(b"vendored")
    return backend


def test_wrapper_places_vendored_weights_from_the_cloned_backend(tmp_path):
    backend = _fixture_checkout(tmp_path, with_weights=True)
    store = tmp_path / "store"

    result = _run_wrapper(tmp_path, backend, store)

    assert "rc=0" in result.stdout, result.stdout + result.stderr
    assert "whisper model: placed into" in result.stdout
    assert (store / "small" / "model.bin").read_bytes() == b"vendored"


def test_wrapper_fails_loudly_and_carries_the_reason(tmp_path):
    backend = _fixture_checkout(tmp_path, with_weights=False)
    store = tmp_path / "store"

    result = _run_wrapper(tmp_path, backend, store)

    assert "rc=1" in result.stdout, result.stdout + result.stderr
    assert "whisper model: provisioning failed" in result.stdout
    assert "downloading is disabled" in result.stderr
    assert "not downloading" not in result.stdout


def test_wrapper_reports_a_missing_helper_instead_of_skipping(tmp_path):
    backend = _fixture_checkout(tmp_path, with_weights=True)
    (backend / "setup" / "installation_scripts" / "provision_whisper_model.py").unlink()

    result = _run_wrapper(tmp_path, backend, tmp_path / "store")

    assert "rc=1" in result.stdout
    assert "provision_whisper_model.py is missing" in result.stdout


def test_setup_runs_the_whisper_step_after_the_clone_and_reports_it():
    text = SETUP_PIB.read_text(encoding="utf-8")

    clone = text.index('run_step "Clone repositories"')
    whisper = text.index('run_step "Provision whisper model" provision_whisper_model')
    containers = text.index("docker_install.sh")
    assert clone < whisper < containers
    assert "voice_assistant.whisper_provision" not in text
    assert "provisioning failed; not downloading" not in text


def test_models_mode_fails_when_the_whisper_step_fails(tmp_path):
    """--models exits non-zero when the voice weights cannot be placed."""
    root = tmp_path / "repo"
    (root / "setup" / "installation_scripts").mkdir(parents=True)
    (root / "setup" / "setup-pib.sh").write_bytes(SETUP_PIB.read_bytes())
    (
        root / "setup" / "installation_scripts" / "resolve_hardware_variant.sh"
    ).write_bytes(
        (
            SETUP_PIB.parent / "installation_scripts" / "resolve_hardware_variant.sh"
        ).read_bytes()
    )
    (
        root / "setup" / "installation_scripts" / "provision_whisper_model.py"
    ).write_bytes(HELPER.read_bytes())
    content = b"blob"
    (root / "models" / "demo").mkdir(parents=True)
    (root / "models" / "demo" / "demo.blob").write_bytes(content)
    (root / "models" / "manifest.yaml").write_text(
        "models:\n- model_id: demo\n  file: demo/demo.blob\n"
        f"  sha256: {hashlib.sha256(content).hexdigest()}\n",
        encoding="utf-8",
    )
    _write_config(root)  # no vendored weights, downloads disabled below
    env = dict(os.environ)
    env.update(
        PIB_MODEL_STORE=str(tmp_path / "store"),
        WHISPER_MODEL_PATH=str(tmp_path / "voice-store"),
        PIB_WHISPER_DOWNLOAD="0",
    )

    result = subprocess.run(
        ["bash", str(root / "setup" / "setup-pib.sh"), "--models"],
        capture_output=True,
        check=False,
        env=env,
        text=True,
    )

    assert result.returncode != 0, result.stdout + result.stderr
    assert "demo: placed" in result.stdout
    assert "whisper model: provisioning failed" in result.stdout
    assert "downloading is disabled" in result.stderr
