"""Regression tests for PR-2332: Ollama / qwen-fast removal and legacy DB retirement."""

from __future__ import annotations

import json
from pathlib import Path

from app.app import db
from commands import _reconcile_model_catalogue
from model.assistant_model import AssistantModel
from model.personality_model import Personality
from model.provider_model import Provider, RegistryModel

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_setup_and_update_scripts_do_not_install_or_modify_ollama():
    """PR-2332: fresh setup skips Ollama/Qwen; updates leave existing host Ollama untouched."""
    for rel in ("setup/setup-pib.sh", "setup/update-pib.sh", "setup/update_runner.sh"):
        text = (REPO_ROOT / rel).read_text(encoding="utf-8")
        assert "install_ollama" not in text, rel
        assert "qwen2.5" not in text, rel
        assert "qwen-fast" not in text, rel
        assert "ollama_listen" not in text, rel
        assert "ensure_ollama_for_update" not in text, rel
        # Existing robots with Ollama already installed must be left untouched.
        assert "systemctl disable ollama" not in text, rel
        assert "systemctl stop ollama" not in text, rel
        assert "rm -rf /usr/local/bin/ollama" not in text, rel

    assert not (REPO_ROOT / "setup/ollama/Modelfile").exists()
    assert not (REPO_ROOT / "setup/installation_scripts/ollama_listen.sh").exists()
    assert not (REPO_ROOT / "pib_api/flask/service/local_model_service.py").exists()
    assert not (
        REPO_ROOT / "pib_hermes_config/pib_hermes_config/local_model.py"
    ).exists()


def test_reconcile_retires_legacy_qwen_fast_row_and_flags_personality(app, app_ctx):
    """Existing robots with a stored qwen-fast personality retire cleanly to needsNewModel."""
    client = app.test_client()
    legacy_provider = Provider(
        name="Local",
        endpoint_base="http://host.docker.internal:11434/v1",
        credential_ref=None,
        capabilities=json.dumps({"tools": False, "images": False}),
    )
    db.session.add(legacy_provider)
    db.session.flush()

    legacy_assistant = AssistantModel(
        api_name="qwen-fast",
        visual_name="Local (qwen-fast)",
        has_image_support=False,
    )
    db.session.add(legacy_assistant)
    db.session.flush()

    legacy_registry = RegistryModel(
        id=legacy_assistant.id,
        provider_id=legacy_provider.id,
        api_name="qwen-fast",
        visual_name="Local (qwen-fast)",
        has_image_support=False,
        capabilities=json.dumps({"offline": True}),
        is_default=False,
    )
    db.session.add(legacy_registry)
    db.session.flush()

    personality = Personality(
        name="LegacyLocal",
        gender="Female",
        pause_threshold=0.8,
        message_history=5,
        assistant_model_id=legacy_assistant.id,
        provider_ref=str(legacy_registry.id),
        channel="direct",
    )
    db.session.add(personality)
    db.session.commit()
    pid = personality.personality_id

    _reconcile_model_catalogue()
    db.session.commit()

    assert RegistryModel.query.filter_by(api_name="qwen-fast").first() is None
    assert AssistantModel.query.filter_by(api_name="qwen-fast").first() is None
    assert Provider.query.filter_by(name="Local").first() is None

    # API must not 500 and must clearly report that the personality needs a new model.
    resp = client.get(f"/voice-assistant/personality/{pid}")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["needsNewModel"] is True
    assert body["assistantModelId"] is None

    # Catalogue never exposes qwen-fast.
    models = client.get("/assistant-model").get_json()["assistantModels"]
    assert all(m["apiName"] != "qwen-fast" for m in models)
