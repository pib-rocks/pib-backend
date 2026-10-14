"""faster-whisper small is the default, and it is not downloaded."""

import sys
import types
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))

from voice_assistant.stt_transcription import (  # noqa: E402
    CPU_THREADS,
    DEFAULT_COMPUTE_TYPE,
    DEFAULT_WHISPER_MODEL_SIZE,
    FasterWhisperSTTEngine,
    resolve_whisper_directory,
)
from voice_assistant.whisper_provision import provision_whisper_model  # noqa: E402


def test_shipped_default_is_small_int8_on_four_threads():
    document = yaml.safe_load(
        (REPO_ROOT / "voice" / "whisper-model.yaml").read_text(encoding="utf-8")
    )
    dockerfile = (VOICE_ROOT / "Dockerfile").read_text(encoding="utf-8")
    camera_manifest = (REPO_ROOT / "models" / "manifest.yaml").read_text(
        encoding="utf-8"
    )
    assert DEFAULT_WHISPER_MODEL_SIZE == "small"
    assert DEFAULT_COMPUTE_TYPE == "int8"
    assert CPU_THREADS == 4
    assert "ENV WHISPER_MODEL_SIZE=small" in dockerfile
    assert "ENV WHISPER_MODEL_SIZE=base" not in dockerfile
    assert document["path"] == "/data/voice/models/whisper/"
    assert document["model_size"] == "small"
    assert document["compute_type"] == "int8"
    assert document["cpu_threads"] == 4
    assert document["medium_is_default"] is False
    assert document["download_at_first_use"] is False
    assert document["first_token_budget_ms"] == 700
    assert document["first_token_budget_rechecked_for_small"] is False
    assert "whisper" not in camera_manifest.lower() or "voice models are not" in (
        camera_manifest.lower()
    )
    assert "model_id: whisper" not in camera_manifest


def test_empty_store_does_not_load_by_model_name(tmp_path, monkeypatch):
    calls = []

    class WhisperModel:
        def __init__(self, identifier, **kwargs):
            calls.append((identifier, kwargs))

    module = types.ModuleType("faster_whisper")
    module.WhisperModel = WhisperModel
    monkeypatch.setitem(sys.modules, "faster_whisper", module)
    monkeypatch.delenv("WHISPER_MODEL_SIZE", raising=False)
    monkeypatch.delenv("WHISPER_COMPUTE_TYPE", raising=False)

    engine = FasterWhisperSTTEngine(model_path=tmp_path)
    assert engine.model_size == "small"
    assert engine.compute_type == "int8"
    assert engine.is_loaded is False
    assert calls == []
    assert resolve_whisper_directory(tmp_path, "small") is None
    assert resolve_whisper_directory(tmp_path, "medium") is None


def test_provisioned_directory_is_what_the_loader_opens(tmp_path, monkeypatch):
    calls = []

    class WhisperModel:
        def __init__(self, identifier, device, compute_type, cpu_threads):
            calls.append(
                {
                    "identifier": identifier,
                    "device": device,
                    "compute_type": compute_type,
                    "cpu_threads": cpu_threads,
                }
            )

    module = types.ModuleType("faster_whisper")
    module.WhisperModel = WhisperModel
    monkeypatch.setitem(sys.modules, "faster_whisper", module)

    flat = tmp_path / "flat"
    flat.mkdir()
    (flat / "model.bin").write_bytes(b"small-weights")
    engine = FasterWhisperSTTEngine(model_path=flat, model_size="small")
    assert engine.is_loaded is True
    assert calls[-1]["identifier"] == str(flat)
    assert calls[-1]["identifier"] != "small"
    assert calls[-1]["compute_type"] == "int8"
    assert calls[-1]["cpu_threads"] == 4

    sized = tmp_path / "sized"
    medium = sized / "medium"
    medium.mkdir(parents=True)
    (medium / "model.bin").write_bytes(b"medium-weights")
    FasterWhisperSTTEngine(model_path=sized, model_size="medium", compute_type="int8")
    assert calls[-1]["identifier"] == str(medium)
    assert calls[-1]["cpu_threads"] == 4


def test_provision_copies_local_weights_and_does_not_invent_them(tmp_path):
    source = tmp_path / "voice" / "whisper" / "small"
    source.mkdir(parents=True)
    (source / "model.bin").write_bytes(b"offline-weights")
    (source / "config.json").write_text("{}", encoding="utf-8")
    destination = tmp_path / "data" / "voice" / "models" / "whisper"

    assert provision_whisper_model(tmp_path / "empty", destination) == "missing"
    assert not (destination / "model.bin").exists()

    assert provision_whisper_model(source.parent, destination) == "placed"
    assert (destination / "small" / "model.bin").read_bytes() == b"offline-weights"
    assert (destination / "small" / "config.json").read_text(encoding="utf-8") == "{}"
    assert provision_whisper_model(source.parent, destination) == "already_current"

    module = (
        REPO_ROOT / "ros_packages/voice_assistant/voice_assistant/whisper_provision.py"
    ).read_text(encoding="utf-8")
    assert "huggingface" not in module
    assert "download" not in module.lower() or "does not download" in module.lower()
