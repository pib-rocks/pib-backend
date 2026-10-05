"""Encrypted key store: one password, wrong password, deletion, no cleartext."""

import json
import logging
import os
import stat
from base64 import urlsafe_b64decode
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pib_hermes_config import profile_dir_for
from pib_hermes_config.token_crypto import (
    TokenCryptoError,
    decrypt_token,
    encrypt_token,
)
from sqlalchemy import text

from app.app import db
from model.personality_model import Personality
from model.provider_model import Provider
from service import key_store_service, personality_service
from service.key_store_service import (
    PASSWORD_CONFIRMATION_MESSAGE,
    PASSWORD_TOO_SHORT_MESSAGE,
    WRONG_PASSWORD_MESSAGE,
    KeyStoreError,
)
from service.soul_service import write_soul

PASSWORD = "operator-secret"
NEW_PASSWORD = "operator-secret-2"
OTHER_PASSWORD = "different-secret"
SECRET = "sk-test-ROUNDTRIP-9f3a"
OTHER_SECRET = "sk-test-SECOND-key-77"
REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def key_store_path(tmp_path, monkeypatch):
    path = tmp_path / "secrets" / "provider_key_store.json"
    monkeypatch.setenv("PIB_KEY_STORE_PATH", str(path))
    monkeypatch.delenv("PIB_UPDATE_DIR", raising=False)
    key_store_service.lock()
    yield path
    key_store_service.lock()


def test_token_crypto_round_trip_and_wrong_password():
    encrypted = encrypt_token("cloud-token", PASSWORD)
    assert encrypted is not None
    salt, ciphertext = encrypted
    assert decrypt_token(PASSWORD, salt, ciphertext) == "cloud-token"
    with pytest.raises(TokenCryptoError, match="Token could not be decrypted"):
        decrypt_token(OTHER_PASSWORD, salt, ciphertext)
    assert encrypt_token("cloud-token", "short") is None
    assert PASSWORD.encode() not in ciphertext
    assert b"cloud-token" not in ciphertext


def test_token_service_uses_the_shared_primitive():
    source = (
        REPO_ROOT / "ros_packages/voice_assistant/voice_assistant/token_service.py"
    ).read_text(encoding="utf-8")
    crypto = (
        REPO_ROOT / "pib_hermes_config/pib_hermes_config/token_crypto.py"
    ).read_text(encoding="utf-8")
    assert "token_crypto.encrypt_token" in source
    assert "token_crypto.decrypt_token" in source
    assert "n=2**14" in crypto


def test_round_trip_uses_one_password_for_every_key(app_ctx, key_store_path):
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    hermes = Provider.query.filter_by(api_name="hermes-agent").one()

    ref = key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    assert ref == f"provider-{vision.id}"
    with pytest.raises(KeyStoreError) as caught:
        key_store_service.put_secret(hermes.id, OTHER_PASSWORD, OTHER_SECRET)
    assert caught.value.status_code == 401
    assert str(caught.value) == WRONG_PASSWORD_MESSAGE

    key_store_service.put_secret(hermes.id, PASSWORD, OTHER_SECRET)
    opened = key_store_service.unlock(PASSWORD)
    assert opened == {
        f"provider-{vision.id}": SECRET,
        f"provider-{hermes.id}": OTHER_SECRET,
    }

    envelope = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert envelope["encrypt_key_storage"] is True
    assert SECRET not in key_store_path.read_text(encoding="utf-8")
    assert OTHER_SECRET not in key_store_path.read_text(encoding="utf-8")
    salt = urlsafe_b64decode(envelope["salt"].encode("ascii"))
    plaintext = decrypt_token(PASSWORD, salt, envelope["ciphertext"].encode("ascii"))
    assert SECRET in plaintext
    assert stat.S_IMODE(key_store_path.stat().st_mode) == 0o600


def test_wrong_password_returns_no_keys_and_does_not_crash(app, app_ctx):
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    db.session.commit()

    response = app.test_client().post(
        "/system/key-store/unlock",
        json={"password": OTHER_PASSWORD},
    )
    assert response.status_code == 401
    body = response.get_json()
    assert body["successful"] is False
    assert body["credentials"] == []
    assert body["error"] == WRONG_PASSWORD_MESSAGE
    assert SECRET not in response.get_data(as_text=True)
    assert key_store_service.unlocked_credentials() == {f"provider-{vision.id}": SECRET}


def test_change_password_creates_the_store_on_a_fresh_robot(
    app, app_ctx, key_store_path
):
    """The store comes into being by setting a password, so the first one must work.

    Regression: the service demanded an existing store file before writing one, which
    made the very first operator password impossible to set - System > Speech answered
    "No encrypted key store exists yet." on a fresh robot.
    """
    assert not key_store_path.exists()
    client = app.test_client()

    created = client.post(
        "/system/key-store/password",
        json={"oldPassword": "", "newPassword": PASSWORD, "confirmPassword": PASSWORD},
    )

    assert created.status_code == 200
    assert created.get_json()["successful"] is True
    assert key_store_path.is_file()
    assert key_store_service.operating_mode() == key_store_service.MODE_UNLOCKED

    # Once the store exists, the old password is required again.
    refused = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": OTHER_PASSWORD,
            "newPassword": NEW_PASSWORD,
            "confirmPassword": NEW_PASSWORD,
        },
    )
    assert refused.status_code == 401
    assert refused.get_json()["error"] == WRONG_PASSWORD_MESSAGE


def test_first_password_must_still_be_long_enough(app, app_ctx, key_store_path):
    """Creating the store does not get to skip the length rule, and writes nothing."""
    client = app.test_client()

    response = client.post(
        "/system/key-store/password",
        json={"oldPassword": "", "newPassword": "short", "confirmPassword": "short"},
    )

    assert response.status_code == 400
    assert response.get_json()["error"] == PASSWORD_TOO_SHORT_MESSAGE
    assert not key_store_path.exists()


def test_change_password_requires_the_new_one_twice(app, app_ctx, key_store_path):
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    client = app.test_client()
    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"password": PASSWORD, "secret": SECRET},
    )
    assert stored.status_code == 200
    before = key_store_path.read_bytes()

    mismatch = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": PASSWORD,
            "newPassword": NEW_PASSWORD,
            "confirmPassword": OTHER_PASSWORD,
        },
    )
    assert mismatch.status_code == 400
    assert mismatch.get_json()["error"] == PASSWORD_CONFIRMATION_MESSAGE
    assert mismatch.get_json()["credentials"] == []
    assert key_store_path.read_bytes() == before
    assert SECRET not in mismatch.get_data(as_text=True)

    changed = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": PASSWORD,
            "newPassword": NEW_PASSWORD,
            "confirmPassword": NEW_PASSWORD,
        },
    )
    assert changed.status_code == 200
    assert changed.get_json() == {"successful": True}
    assert SECRET not in changed.get_data(as_text=True)

    old = client.post("/system/key-store/unlock", json={"password": PASSWORD})
    assert old.status_code == 401
    assert old.get_json()["credentials"] == []
    opened = key_store_service.unlock(NEW_PASSWORD)
    assert opened[f"provider-{vision.id}"] == SECRET

    too_short = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": NEW_PASSWORD,
            "newPassword": "short",
            "confirmPassword": "short",
        },
    )
    assert too_short.status_code == 400
    assert too_short.get_json()["error"] == PASSWORD_TOO_SHORT_MESSAGE
    assert key_store_service.unlock(NEW_PASSWORD)[f"provider-{vision.id}"] == SECRET


def test_deleting_a_key_leaves_the_personality_reference(
    app_ctx, key_store_path, monkeypatch
):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    hermes = Provider.query.filter_by(api_name="hermes-agent").one()
    personality = personality_service.create_personality(
        {
            "name": "OrphanRef",
            "gender": "Female",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": vision.id,
        }
    )
    original_ref = personality.provider_ref
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    key_store_service.put_secret(hermes.id, PASSWORD, OTHER_SECRET)

    key_store_service.delete_secret(vision.id, PASSWORD)

    reloaded = Personality.query.filter_by(
        personality_id=personality.personality_id
    ).one()
    assert reloaded.provider_ref == original_ref
    from service import provider_service

    assert provider_service.resolve_provider(reloaded.provider_ref).id == vision.id
    db.session.refresh(vision)
    assert vision.credential_ref is None
    opened = key_store_service.unlock(PASSWORD)
    assert SECRET not in opened.values()
    assert opened[f"provider-{hermes.id}"] == OTHER_SECRET
    assert SECRET.encode() not in key_store_path.read_bytes()

    with pytest.raises(KeyStoreError) as caught:
        key_store_service.delete_secret(hermes.id, OTHER_PASSWORD)
    assert caught.value.status_code == 401
    db.session.refresh(hermes)
    assert hermes.credential_ref == f"provider-{hermes.id}"


def test_credentials_stay_out_of_soul_memory_logs_and_the_database(
    app, app_ctx, key_store_path, monkeypatch, caplog
):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    caplog.set_level(logging.DEBUG)
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    personality = personality_service.create_personality(
        {
            "name": "Cleartext",
            "gender": "Male",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": vision.id,
            "description": "A calm robot.",
        }
    )
    write_soul(personality.personality_id, personality.description, personality.name)
    memory_path = (
        Path(profile_dir_for(personality.personality_id)) / "memories" / "MEMORY.md"
    )
    memory_path.parent.mkdir(parents=True, exist_ok=True)
    memory_path.write_text("remember the colour blue\n", encoding="utf-8")
    soul_before = Path(
        os.path.join(profile_dir_for(personality.personality_id), "SOUL.md")
    ).read_text(encoding="utf-8")
    db.session.commit()

    client = app.test_client()
    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"password": PASSWORD, "secret": SECRET},
    )
    assert stored.status_code == 200
    wrong = client.post(
        "/system/key-store/unlock",
        json={"password": OTHER_PASSWORD},
    )
    assert wrong.status_code == 401
    status = client.get("/system/key-store")
    provider = client.get(f"/provider/{vision.id}")

    assert status.get_json()["encryptKeyStorage"] is True
    assert status.get_json()["credentialRefs"] == [f"provider-{vision.id}"]
    assert provider.get_json()["credentialRef"] == f"provider-{vision.id}"
    for response in (stored, wrong, status, provider):
        payload = response.get_data(as_text=True)
        assert SECRET not in payload
        assert PASSWORD not in payload

    db.session.commit()
    db.session.execute(text("PRAGMA wal_checkpoint(TRUNCATE)"))
    db_path = app.config["SQLALCHEMY_DATABASE_URI"].removeprefix("sqlite:///")
    blob = Path(db_path).read_bytes()
    for suffix in ("-wal", "-shm"):
        extra = Path(db_path + suffix)
        if extra.exists():
            blob += extra.read_bytes()
    assert SECRET.encode() not in blob
    assert PASSWORD.encode() not in blob

    profiles = Path(os.environ["PIB_HERMES_PROFILES_DIR"])
    tree = b"".join(path.read_bytes() for path in profiles.rglob("*") if path.is_file())
    assert SECRET.encode() not in tree
    assert PASSWORD.encode() not in tree
    assert memory_path.read_text(encoding="utf-8") == "remember the colour blue\n"
    soul_path = Path(profile_dir_for(personality.personality_id)) / "SOUL.md"
    assert soul_path.read_text(encoding="utf-8") == soul_before
    assert SECRET not in caplog.text
    assert PASSWORD not in caplog.text
    assert SECRET.encode() not in key_store_path.read_bytes()
    assert PASSWORD.encode() not in key_store_path.read_bytes()


def test_short_password_does_not_create_a_store(app, app_ctx, key_store_path):
    vision = Provider.query.filter_by(visual_name="GPT-4o [Vision]").one()
    response = app.test_client().put(
        f"/system/key-store/{vision.id}",
        json={"password": "short", "secret": SECRET},
    )
    assert response.status_code == 400
    assert response.get_json()["error"] == PASSWORD_TOO_SHORT_MESSAGE
    assert response.get_json()["credentials"] == []
    assert not key_store_path.exists()
    db.session.refresh(vision)
    assert vision.credential_ref is None


def test_status_reports_encryption_on_before_any_key(app):
    body = app.test_client().get("/system/key-store").get_json()
    assert body == {
        "encryptKeyStorage": True,
        "credentialRefs": [],
        "mode": "degraded",
    }
