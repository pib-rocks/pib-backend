"""The SmartConnect token is one key-store entry, opened by the one password."""

import json
import logging
import sys
from pathlib import Path

import pytest

from model.provider_model import RegistryModel
from service import key_store_service

PASSWORD = "operator-secret"
TOKEN = "sc-test-CLOUD-token-91"
OTHER = "sk-test-OTHER-provider-4c"
CLOUD_MESSAGE = "No keys are available for provider pib-cloud."
REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))


@pytest.fixture(autouse=True)
def locked_store():
    key_store_service.lock()
    yield
    key_store_service.lock()


def _visible(caplog, capsys) -> str:
    captured = capsys.readouterr()
    return caplog.text + captured.out + captured.err


def _encryption_off() -> None:
    path = Path(key_store_service.settings_path())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('{"encrypt_key_storage": false}', encoding="utf-8")


def _set_password(client) -> None:
    created = client.post(
        "/system/key-store/password",
        json={
            "oldPassword": "",
            "newPassword": PASSWORD,
            "confirmPassword": PASSWORD,
        },
    )
    assert created.status_code == 200


def _unlock(client) -> None:
    opened = client.post("/system/key-store/unlock", json={"password": PASSWORD})
    assert opened.status_code == 200


def _store_text() -> str:
    path = Path(key_store_service.store_path())
    if not path.is_file():
        return ""
    return path.read_text(encoding="utf-8")


def test_unlocked_store_supplies_the_cloud_token_and_locked_does_not(
    app, app_ctx, caplog, capsys
):
    """Both branches: the open store returns the token, the locked store does not."""
    caplog.set_level(logging.INFO)
    client = app.test_client()
    _set_password(client)
    other = RegistryModel.query.filter_by(api_name="gpt-6").one().provider
    stored_other = client.put(
        f"/system/key-store/{other.id}",
        json={"password": PASSWORD, "secret": OTHER},
    )
    assert stored_other.status_code == 200

    key_store_service.lock()
    refused = client.post("/system/smart-connect", json={"token": TOKEN})
    assert refused.status_code == 401
    assert refused.get_json()["error"] == CLOUD_MESSAGE
    assert "credentialRef" not in refused.get_json()
    assert TOKEN not in refused.get_data(as_text=True)
    assert TOKEN not in _store_text()
    from voice_assistant.cloud_token import unavailable_message

    assert refused.get_json()["error"] == unavailable_message()

    _unlock(client)
    accepted = client.post(
        "/system/smart-connect",
        json={"token": TOKEN, "password": "not-the-operator-password"},
    )
    assert accepted.status_code == 200
    body = accepted.get_json()
    assert set(body) == {"successful", "credentialRef"}
    assert body["successful"] is True
    assert body["credentialRef"].startswith("provider-")
    assert TOKEN not in accepted.get_data(as_text=True)
    assert TOKEN not in _store_text()
    assert PASSWORD not in _store_text()
    assert "not-the-operator-password" not in _store_text()

    cloud = RegistryModel.query.filter_by(api_name="pib-cloud").one()
    assert cloud.provider.credential_ref == body["credentialRef"]
    assert TOKEN not in (cloud.provider.credential_ref or "")

    opened = client.get("/system/key-store/credential/pib-cloud")
    assert opened.status_code == 200
    assert opened.get_json()["mode"] == "unlocked"
    assert opened.get_json()["available"] is True
    assert opened.get_json()["secret"] == TOKEN
    other_key = client.get("/system/key-store/credential/gpt-6")
    assert other_key.get_json()["secret"] == OTHER
    assert TOKEN not in other_key.get_data(as_text=True)

    from voice_assistant.cloud_token import CloudTokenError, resolve_cloud_token

    asked = []

    def fetch():
        response = client.get("/system/key-store/credential/pib-cloud")
        payload = response.get_json()
        asked.append(payload.get("mode"))
        secret = payload.get("secret") if payload.get("available") is True else None
        return {"mode": payload.get("mode"), "secret": secret}

    token = resolve_cloud_token(fetch)
    assert token == TOKEN
    assert asked == ["unlocked"]
    assert "cloud token source=key-store provider=pib-cloud" in caplog.text

    key_store_service.lock()
    locked = client.get("/system/key-store/credential/pib-cloud")
    assert locked.get_json() == {"mode": "degraded", "available": False}
    assert TOKEN not in locked.get_data(as_text=True)

    def locked_fetch():
        return {"mode": "degraded", "secret": TOKEN}

    with pytest.raises(CloudTokenError) as caught:
        resolve_cloud_token(locked_fetch)
    assert str(caught.value) == CLOUD_MESSAGE
    visible = _visible(caplog, capsys) + str(caught.value)
    assert TOKEN not in visible
    assert OTHER not in visible
    assert PASSWORD not in visible
    assert "not-the-operator-password" not in visible


def test_cleartext_mode_stores_the_token_without_a_password(
    app, app_ctx, caplog, capsys, monkeypatch
):
    """Encryption off never asks for a password, and the token still arrives."""
    caplog.set_level(logging.INFO)
    monkeypatch.setenv("GOOGLE_API_KEY", OTHER)
    monkeypatch.setenv("GEMINI_API_KEY", "env-token-should-not-be-used")
    _encryption_off()
    key_store_service.lock()
    assert key_store_service.operating_mode() == "unlocked"
    client = app.test_client()

    accepted = client.post("/system/smart-connect", json={"token": TOKEN})
    assert accepted.status_code == 200
    body = accepted.get_json()
    assert body["credentialRef"].startswith("provider-")
    assert TOKEN not in accepted.get_data(as_text=True)
    assert "password" not in accepted.get_data(as_text=True).lower()

    opened = client.get("/system/key-store/credential/pib-cloud")
    assert opened.get_json()["secret"] == TOKEN
    status = client.get("/system/key-store")
    assert status.get_json()["mode"] == "unlocked"
    assert status.get_json()["encryptKeyStorage"] is False
    assert TOKEN not in status.get_data(as_text=True)

    from voice_assistant.cloud_token import resolve_cloud_token

    def fetch():
        payload = client.get("/system/key-store/credential/pib-cloud").get_json()
        return {
            "mode": payload.get("mode"),
            "secret": payload.get("secret") if payload.get("available") else None,
        }

    assert resolve_cloud_token(fetch) == TOKEN
    assert "cloud token source=key-store provider=pib-cloud" in caplog.text
    visible = _visible(caplog, capsys)
    assert TOKEN not in visible
    assert OTHER not in visible


def test_locked_or_other_provider_payload_is_not_the_cloud_token(caplog, capsys):
    """A secret on a locked payload, or another provider, is not returned."""
    from voice_assistant.cloud_token import CloudTokenError, read_cloud_token

    caplog.set_level(logging.INFO)

    def degraded():
        return {"mode": "degraded", "secret": TOKEN}

    with pytest.raises(CloudTokenError) as locked:
        read_cloud_token(degraded)
    assert str(locked.value) == CLOUD_MESSAGE

    def missing():
        return {"mode": "unlocked", "secret": None}

    with pytest.raises(CloudTokenError) as empty:
        read_cloud_token(missing)
    assert str(empty.value) == CLOUD_MESSAGE

    visible = _visible(caplog, capsys) + str(locked.value) + str(empty.value)
    assert TOKEN not in visible


def test_submit_sends_the_token_and_no_password(caplog, capsys):
    """The voice node posts the token alone and does not log it."""
    from voice_assistant.cloud_token import submit_cloud_token

    caplog.set_level(logging.INFO)
    captured = {}

    class Response:
        status = 200

        def read(self):
            return b'{"successful": true, "credentialRef": "provider-4"}'

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def opener(request, timeout):
        del timeout
        captured["body"] = request.data
        captured["url"] = request.full_url
        return Response()

    ref = submit_cloud_token(TOKEN, opener=opener)
    assert ref == "provider-4"
    assert captured["url"].endswith("/system/smart-connect")
    sent = json.loads(captured["body"].decode("utf-8"))
    assert set(sent) == {"token"}
    assert sent["token"] == TOKEN
    visible = _visible(caplog, capsys)
    assert TOKEN not in visible
