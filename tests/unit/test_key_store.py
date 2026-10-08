"""Key store: one password while encryption is on, cleartext when it is off."""

import json
import logging
import os
import stat
from base64 import urlsafe_b64decode
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pib_hermes_config import profile_dir_for
from sqlalchemy import text

from app.app import db
from model.personality_model import Personality
from model.provider_model import RegistryModel
from service import key_store_service, personality_service
from service.key_store_service import (
    PASSWORD_CONFIRMATION_MESSAGE,
    PASSWORD_TOO_SHORT_MESSAGE,
    UNREADABLE_MESSAGE,
    UNWRITABLE_MESSAGE,
    WRONG_PASSWORD_MESSAGE,
    KeyStoreError,
    TokenCryptoError,
    decrypt_token,
    encrypt_token,
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


def test_token_service_reads_the_key_store_not_a_second_cipher():
    """The voice node reads the cloud token from the key store."""
    source = (
        REPO_ROOT / "ros_packages/voice_assistant/voice_assistant/token_service.py"
    ).read_text(encoding="utf-8")
    crypto = (REPO_ROOT / "pib_api/flask/service/key_store_service.py").read_text(
        encoding="utf-8"
    )
    assert "token_crypto" not in source
    assert "read_cloud_token" in source
    assert "log_cloud_token_source" in source
    assert "n=2**14" in crypto


def test_round_trip_uses_one_password_for_every_key(app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    second = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one().provider

    ref = key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    assert ref == f"provider-{vision.id}"
    with pytest.raises(KeyStoreError) as caught:
        key_store_service.put_secret(second.id, OTHER_PASSWORD, OTHER_SECRET)
    assert caught.value.status_code == 401
    assert str(caught.value) == WRONG_PASSWORD_MESSAGE

    key_store_service.put_secret(second.id, PASSWORD, OTHER_SECRET)
    opened = key_store_service.unlock(PASSWORD)
    assert opened == {
        f"provider-{vision.id}": SECRET,
        f"provider-{second.id}": OTHER_SECRET,
    }

    envelope = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert envelope["encrypt_key_storage"] is True
    assert SECRET not in key_store_path.read_text(encoding="utf-8")
    assert OTHER_SECRET not in key_store_path.read_text(encoding="utf-8")
    salt = urlsafe_b64decode(envelope["salt"].encode("ascii"))
    plaintext = decrypt_token(PASSWORD, salt, envelope["ciphertext"].encode("ascii"))
    assert SECRET in plaintext
    assert stat.S_IMODE(key_store_path.stat().st_mode) == 0o600


def test_credential_route_returns_one_unlocked_key_and_never_logs_it(
    app, app_ctx, caplog
):
    """The voice node reads one provider. A locked store and the status route do not."""
    caplog.set_level(logging.INFO)
    gemini = RegistryModel.query.filter_by(api_name="gemini-3.8-flash").one().provider
    other = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    key_store_service.put_secret(gemini.id, PASSWORD, SECRET)
    key_store_service.put_secret(other.id, PASSWORD, OTHER_SECRET)
    db.session.commit()
    client = app.test_client()

    opened = client.get("/system/key-store/credential/gemini-3.8-flash")
    assert opened.status_code == 200
    body = opened.get_json()
    assert body["mode"] == "unlocked"
    assert body["available"] is True
    assert body["secret"] == SECRET
    assert OTHER_SECRET not in opened.get_data(as_text=True)

    status = client.get("/system/key-store")
    assert SECRET not in status.get_data(as_text=True)
    assert OTHER_SECRET not in status.get_data(as_text=True)

    untouched = client.get("/system/key-store/credential/claude-sonnet-5-5")
    assert untouched.status_code == 200
    assert untouched.get_json()["available"] is False
    assert "secret" not in untouched.get_json()
    assert SECRET.encode() not in untouched.get_data()
    assert OTHER_SECRET.encode() not in untouched.get_data()

    key_store_service.lock()
    locked = client.get("/system/key-store/credential/gemini-3.8-flash")
    assert locked.status_code == 200
    assert locked.get_json() == {"mode": "degraded", "available": False}
    assert SECRET not in locked.get_data(as_text=True)
    assert SECRET not in caplog.text
    assert OTHER_SECRET not in caplog.text


def test_wrong_password_returns_no_keys_and_does_not_crash(app, app_ctx):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
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
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
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
    gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
    vision = gpt6.provider
    second = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one().provider
    personality = personality_service.create_personality(
        {
            "name": "OrphanRef",
            "gender": "Female",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": gpt6.id,
        }
    )
    original_ref = personality.provider_ref
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    key_store_service.put_secret(second.id, PASSWORD, OTHER_SECRET)

    key_store_service.delete_secret(vision.id, PASSWORD)

    reloaded = Personality.query.filter_by(
        personality_id=personality.personality_id
    ).one()
    assert reloaded.provider_ref == original_ref
    from service import provider_service

    assert provider_service.resolve_model(reloaded.provider_ref).id == gpt6.id
    assert provider_service.resolve_provider(reloaded.provider_ref).id == vision.id
    db.session.refresh(vision)
    assert vision.credential_ref is None
    opened = key_store_service.unlock(PASSWORD)
    assert SECRET not in opened.values()
    assert opened[f"provider-{second.id}"] == OTHER_SECRET
    assert SECRET.encode() not in key_store_path.read_bytes()

    with pytest.raises(KeyStoreError) as caught:
        key_store_service.delete_secret(second.id, OTHER_PASSWORD)
    assert caught.value.status_code == 401
    db.session.refresh(second)
    assert second.credential_ref == f"provider-{second.id}"


def test_credentials_stay_out_of_soul_memory_logs_and_the_database(
    app, app_ctx, key_store_path, monkeypatch, caplog
):
    monkeypatch.setattr(
        personality_service,
        "_provision_profile",
        MagicMock(return_value={"ok": True}),
    )
    caplog.set_level(logging.DEBUG)
    gpt6 = RegistryModel.query.filter_by(api_name="gpt-6").one()
    vision = gpt6.provider
    personality = personality_service.create_personality(
        {
            "name": "Cleartext",
            "gender": "Male",
            "pause_threshold": 0.8,
            "message_history": 5,
            "assistant_model_id": gpt6.id,
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
    provider = client.get(f"/provider/{gpt6.id}")

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
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
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


def _settings_file(key_store_path):
    return key_store_path.parent / "key_store_settings.json"


def test_settings_path_follows_the_store_unless_overridden(key_store_path, monkeypatch):
    settings_path = getattr(key_store_service, "settings_path", None)
    assert settings_path is not None
    assert settings_path() == str(_settings_file(key_store_path))
    override = key_store_path.parent / "custom-settings.json"
    monkeypatch.setenv("PIB_KEY_STORE_SETTINGS_PATH", str(override))
    assert key_store_service.settings_path() == str(override)


def test_cold_start_stores_a_first_key_with_encryption_on_and_off(
    app, app_ctx, key_store_path
):
    """No store and no settings file. On needs a password. Off does not.

    A missing settings file means encryption stays on.
    """
    settings = _settings_file(key_store_path)
    assert not key_store_path.exists()
    assert not settings.exists()
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    second = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one().provider
    client = app.test_client()

    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"password": PASSWORD, "secret": SECRET},
    )
    assert stored.status_code == 200
    assert key_store_service.unlock(PASSWORD)[f"provider-{vision.id}"] == SECRET
    status = client.get("/system/key-store").get_json()
    assert status["encryptKeyStorage"] is True
    assert not settings.exists()
    assert SECRET.encode() not in key_store_path.read_bytes()

    missing_password = client.put(
        f"/system/key-store/{second.id}",
        json={"secret": OTHER_SECRET},
    )
    assert missing_password.status_code == 400

    key_store_path.unlink()
    key_store_service.lock()
    assert not key_store_path.exists()
    assert not settings.exists()

    switched = client.post("/system/key-store/encryption", json={"enabled": False})
    assert switched.status_code == 200
    assert switched.get_json()["encryptKeyStorage"] is False
    assert switched.get_json()["mode"] == "unlocked"
    assert not key_store_path.exists()

    clear = client.put(
        f"/system/key-store/{second.id}",
        json={"secret": OTHER_SECRET},
    )
    assert clear.status_code == 200
    document = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert document["cleartext"] is True
    assert "ciphertext" not in document
    assert "salt" not in document
    assert document["secrets"][f"provider-{second.id}"] == OTHER_SECRET
    assert stat.S_IMODE(key_store_path.stat().st_mode) == 0o600
    key_store_service.lock()
    assert key_store_service.operating_mode() == "unlocked"
    assert (
        key_store_service.unlocked_credentials()[f"provider-{second.id}"]
        == OTHER_SECRET
    )
    opened = client.post("/system/key-store/unlock", json={})
    assert opened.status_code == 200
    assert opened.get_json()["mode"] == "unlocked"
    assert OTHER_SECRET not in opened.get_data(as_text=True)

    removed = client.delete(f"/system/key-store/{second.id}", json={})
    assert removed.status_code == 204
    db.session.refresh(second)
    assert second.credential_ref is None


def test_switching_encryption_off_and_on_keeps_the_keys(app, app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    second = RegistryModel.query.filter_by(api_name="claude-sonnet-5-5").one().provider
    client = app.test_client()
    assert (
        client.put(
            f"/system/key-store/{vision.id}",
            json={"password": PASSWORD, "secret": SECRET},
        ).status_code
        == 200
    )
    assert (
        client.put(
            f"/system/key-store/{second.id}",
            json={"password": PASSWORD, "secret": OTHER_SECRET},
        ).status_code
        == 200
    )

    turned_off = client.post(
        "/system/key-store/encryption",
        json={"enabled": False, "password": PASSWORD},
    )
    assert turned_off.status_code == 200
    assert turned_off.get_json()["encryptKeyStorage"] is False
    assert turned_off.get_json()["mode"] == "unlocked"
    assert SECRET not in turned_off.get_data(as_text=True)
    document = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert document["cleartext"] is True
    assert document["secrets"] == {
        f"provider-{vision.id}": SECRET,
        f"provider-{second.id}": OTHER_SECRET,
    }
    assert "ciphertext" not in document
    settings = json.loads(_settings_file(key_store_path).read_text(encoding="utf-8"))
    assert settings == {"encrypt_key_storage": False}
    key_store_service.lock()
    assert key_store_service.unlocked_credentials() == {
        f"provider-{vision.id}": SECRET,
        f"provider-{second.id}": OTHER_SECRET,
    }
    # Mode unlocked is what the start-up prompt reads, so the prompt stays down.
    assert client.get("/system/key-store").get_json()["mode"] == "unlocked"

    clear_bytes = key_store_path.read_bytes()
    refused = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": "",
            "newPassword": NEW_PASSWORD,
            "confirmPassword": NEW_PASSWORD,
        },
    )
    assert refused.status_code == 400
    assert refused.get_json()["error"] == key_store_service.ENCRYPTION_OFF_MESSAGE
    assert key_store_path.read_bytes() == clear_bytes

    turned_on = client.post(
        "/system/key-store/encryption",
        json={"enabled": True, "password": NEW_PASSWORD},
    )
    assert turned_on.status_code == 200
    assert turned_on.get_json()["encryptKeyStorage"] is True
    envelope = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert envelope["encrypt_key_storage"] is True
    assert envelope.get("cleartext") is not True
    assert "salt" in envelope
    assert "ciphertext" in envelope
    raw = key_store_path.read_text(encoding="utf-8")
    assert SECRET not in raw
    assert OTHER_SECRET not in raw
    assert NEW_PASSWORD not in raw
    settings = json.loads(_settings_file(key_store_path).read_text(encoding="utf-8"))
    assert settings == {"encrypt_key_storage": True}
    key_store_service.lock()
    assert key_store_service.operating_mode() == "degraded"
    opened = key_store_service.unlock(NEW_PASSWORD)
    assert opened[f"provider-{vision.id}"] == SECRET
    assert opened[f"provider-{second.id}"] == OTHER_SECRET
    with pytest.raises(KeyStoreError) as caught:
        key_store_service.unlock(PASSWORD)
    assert caught.value.status_code == 401


def test_empty_store_turns_encryption_off_without_a_password(
    app, app_ctx, key_store_path
):
    """Setting a password creates a file even when no key was stored.

    That file is an encrypted empty map. Turning encryption off used to
    demand the operator password because the file existed. There is nothing
    to decrypt, and the next start must not open the password prompt.
    """
    client = app.test_client()
    created = client.post(
        "/system/key-store/password",
        json={"oldPassword": "", "newPassword": PASSWORD, "confirmPassword": PASSWORD},
    )
    assert created.status_code == 200
    assert key_store_path.is_file()
    before = key_store_path.read_bytes()
    assert SECRET.encode() not in before
    settings = _settings_file(key_store_path)
    assert not settings.exists()
    key_store_service.lock()
    assert key_store_service.operating_mode() == "degraded"

    turned_off = client.post("/system/key-store/encryption", json={"enabled": False})
    assert turned_off.status_code == 200
    body = turned_off.get_json()
    assert body["encryptKeyStorage"] is False
    assert body["mode"] == "unlocked"
    assert key_store_service.operating_mode() == "unlocked"
    assert PASSWORD not in turned_off.get_data(as_text=True)
    document = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert document["cleartext"] is True
    assert document["secrets"] == {}
    assert "ciphertext" not in document
    assert "salt" not in document
    assert json.loads(settings.read_text(encoding="utf-8")) == {
        "encrypt_key_storage": False
    }

    # A restarted process keeps nothing but the files. Unlocked is what the
    # start-up prompt reads, so the prompt stays down.
    key_store_service.lock()
    assert key_store_service.operating_mode() == "unlocked"
    status = client.get("/system/key-store").get_json()
    assert status["encryptKeyStorage"] is False
    assert status["credentialRefs"] == []
    assert status["mode"] == "unlocked"


def test_empty_store_keeps_encryption_when_the_rewrite_fails(
    app, app_ctx, key_store_path, monkeypatch
):
    """The settings file is written only after the empty store is rewritten."""
    client = app.test_client()
    created = client.post(
        "/system/key-store/password",
        json={"oldPassword": "", "newPassword": PASSWORD, "confirmPassword": PASSWORD},
    )
    assert created.status_code == 200
    before = key_store_path.read_bytes()
    settings = _settings_file(key_store_path)
    assert not settings.exists()
    key_store_service.lock()
    real_replace = key_store_service.os.replace

    def fail_store_replace(src, dst):
        if os.path.abspath(dst) == os.path.abspath(str(key_store_path)):
            raise OSError("store replace failed")
        return real_replace(src, dst)

    monkeypatch.setattr(key_store_service.os, "replace", fail_store_replace)
    refused = client.post("/system/key-store/encryption", json={"enabled": False})
    assert refused.status_code == 500
    assert refused.get_json()["error"] == UNWRITABLE_MESSAGE
    assert not settings.exists()
    assert key_store_path.read_bytes() == before
    assert client.get("/system/key-store").get_json()["encryptKeyStorage"] is True
    assert key_store_service.operating_mode() == "degraded"


def test_wrong_password_refuses_turning_encryption_off(app, app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    client = app.test_client()
    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"password": PASSWORD, "secret": SECRET},
    )
    assert stored.status_code == 200
    before = key_store_path.read_bytes()
    settings = _settings_file(key_store_path)
    assert not settings.exists()

    refused = client.post(
        "/system/key-store/encryption",
        json={"enabled": False, "password": OTHER_PASSWORD},
    )
    assert refused.status_code == 401
    assert refused.get_json()["error"] == WRONG_PASSWORD_MESSAGE
    assert refused.get_json()["credentials"] == []
    assert SECRET not in refused.get_data(as_text=True)
    assert PASSWORD not in refused.get_data(as_text=True)
    assert key_store_path.read_bytes() == before
    assert not settings.exists()
    assert SECRET.encode() not in key_store_path.read_bytes()
    assert json.loads(before)["encrypt_key_storage"] is True

    # The same request the empty-store case sends. A real entry still refuses it.
    omitted = client.post("/system/key-store/encryption", json={"enabled": False})
    assert omitted.status_code == 401
    assert omitted.get_json()["error"] == WRONG_PASSWORD_MESSAGE
    assert omitted.get_json()["credentials"] == []
    assert SECRET not in omitted.get_data(as_text=True)
    assert key_store_path.read_bytes() == before
    assert not settings.exists()


def test_short_password_refuses_turning_encryption_on(app, app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    client = app.test_client()
    switched = client.post("/system/key-store/encryption", json={"enabled": False})
    assert switched.status_code == 200
    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"secret": SECRET},
    )
    assert stored.status_code == 200
    before_store = key_store_path.read_bytes()
    before_settings = _settings_file(key_store_path).read_bytes()

    refused = client.post(
        "/system/key-store/encryption",
        json={"enabled": True, "password": "short"},
    )
    assert refused.status_code == 400
    assert refused.get_json()["error"] == PASSWORD_TOO_SHORT_MESSAGE
    assert key_store_path.read_bytes() == before_store
    assert _settings_file(key_store_path).read_bytes() == before_settings
    assert json.loads(before_settings)["encrypt_key_storage"] is False
    assert json.loads(key_store_path.read_text(encoding="utf-8"))["cleartext"] is True
    key_store_service.lock()
    assert key_store_service.unlocked_credentials()[f"provider-{vision.id}"] == SECRET


def test_failed_rewrite_does_not_persist_the_encryption_setting(
    app, app_ctx, key_store_path, monkeypatch
):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    client = app.test_client()
    stored = client.put(
        f"/system/key-store/{vision.id}",
        json={"password": PASSWORD, "secret": SECRET},
    )
    assert stored.status_code == 200
    before = key_store_path.read_bytes()
    settings = _settings_file(key_store_path)
    assert not settings.exists()
    real_replace = key_store_service.os.replace

    def fail_store_replace(src, dst):
        if os.path.abspath(dst) == os.path.abspath(str(key_store_path)):
            raise OSError("store replace failed")
        return real_replace(src, dst)

    monkeypatch.setattr(key_store_service.os, "replace", fail_store_replace)
    refused = client.post(
        "/system/key-store/encryption",
        json={"enabled": False, "password": PASSWORD},
    )
    assert refused.status_code == 500
    assert refused.get_json()["error"] == UNWRITABLE_MESSAGE
    assert not settings.exists()
    assert key_store_path.read_bytes() == before
    assert client.get("/system/key-store").get_json()["encryptKeyStorage"] is True


def test_status_reports_the_encryption_setting(app, key_store_path):
    client = app.test_client()
    settings = _settings_file(key_store_path)
    settings.parent.mkdir(parents=True, exist_ok=True)
    settings.write_text("{", encoding="utf-8")
    unreadable = client.get("/system/key-store").get_json()
    assert unreadable["encryptKeyStorage"] is True
    assert unreadable["mode"] == "degraded"

    settings.write_text('{"encrypt_key_storage": false}', encoding="utf-8")
    body = client.get("/system/key-store").get_json()
    assert body["encryptKeyStorage"] is False
    assert body["mode"] == "unlocked"
    assert body["credentialRefs"] == []
    assert set(body) == {"encryptKeyStorage", "credentialRefs", "mode"}
    assert SECRET not in client.get("/system/key-store").get_data(as_text=True)


def test_cleartext_keys_need_no_unlock(app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    ref = f"provider-{vision.id}"
    key_store_path.parent.mkdir(parents=True, exist_ok=True)
    key_store_path.write_text(
        json.dumps(
            {"cleartext": True, "secrets": {ref: SECRET}, "version": 1},
            separators=(",", ":"),
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    _settings_file(key_store_path).write_text(
        '{"encrypt_key_storage": false}', encoding="utf-8"
    )
    key_store_service.lock()
    assert key_store_service.operating_mode() == "unlocked"
    assert key_store_service.unlocked_credentials()[ref] == SECRET
    assert key_store_service.unlock("")[ref] == SECRET


def test_cleartext_file_is_not_read_as_an_envelope(app_ctx, key_store_path):
    """A cleartext mark blocks envelope reading, even if salt is also present."""
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    envelope = json.loads(key_store_path.read_text(encoding="utf-8"))
    envelope["cleartext"] = True
    key_store_path.write_text(
        json.dumps(envelope, separators=(",", ":"), sort_keys=True),
        encoding="utf-8",
    )
    key_store_service.lock()
    with pytest.raises(KeyStoreError) as caught:
        key_store_service.unlock(PASSWORD)
    assert caught.value.status_code == 422
    assert str(caught.value) == UNREADABLE_MESSAGE
    loaded = json.loads(key_store_path.read_text(encoding="utf-8"))
    assert loaded["cleartext"] is True
    assert SECRET.encode() not in key_store_path.read_bytes()


def test_envelope_is_not_read_as_cleartext(app_ctx, key_store_path):
    vision = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    key_store_service.put_secret(vision.id, PASSWORD, SECRET)
    _settings_file(key_store_path).parent.mkdir(parents=True, exist_ok=True)
    _settings_file(key_store_path).write_text(
        '{"encrypt_key_storage": false}', encoding="utf-8"
    )
    before = key_store_path.read_bytes()
    key_store_service.lock()
    with pytest.raises(KeyStoreError) as caught:
        key_store_service.unlock(PASSWORD)
    assert caught.value.status_code == 422
    assert str(caught.value) == UNREADABLE_MESSAGE
    assert key_store_path.read_bytes() == before
    assert SECRET.encode() not in key_store_path.read_bytes()
