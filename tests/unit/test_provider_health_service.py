"""PC-115: the provider health probe behind the voice-assistant status strip.

Two behaviours carry the requirement:

* pib.Cloud is green only when the service answers, the stored SmartConnect
  token is accepted and the token still has budget. A live ``/health`` alone
  is not enough.
* A probe that cannot run is ``unknown`` (neutral in the UI), never
  ``unreachable``: a missing key or a catalogue row without an endpoint must
  not look like an outage.
"""

from __future__ import annotations

import pytest

from model.provider_model import Provider, RegistryModel
from service import key_store_service, provider_health_service as health

CLOUD_MODEL_ID = 5


def _cloud_row(app_ctx) -> RegistryModel:
    return RegistryModel.query.filter_by(api_name="pib-cloud").one()


def _stored_secret(monkeypatch, value):
    """Pretend the key store is unlocked and holds this secret for the row."""
    monkeypatch.setattr(
        key_store_service, "unlocked_secret_for_api_name", lambda api_name: value
    )


def _responses(monkeypatch, table):
    """Replace the HTTP layer. ``table`` maps a URL suffix to (status, body)."""
    calls = []

    def fake_get(url, headers):
        calls.append((url, headers))
        for suffix, (status, body) in table.items():
            if url.endswith(suffix):
                error = "" if status is not None else "URLError: down"
                return status, body, error, 7
        raise AssertionError(f"unexpected probe URL: {url}")

    monkeypatch.setattr(health, "_http_get", fake_get)
    return calls


def _fake_model(provider_name, api_name, endpoint_base):
    """A catalogue row without a database."""
    return _FakeModel(
        Provider(name=provider_name, capabilities={}, endpoint_base=endpoint_base),
        api_name=api_name,
        visual_name=f"{provider_name} {api_name}",
    )


def _serve(monkeypatch, model):
    monkeypatch.setattr(health.provider_service, "get_model_by_id", lambda _id: model)


# --- pib.Cloud: reachable AND token valid AND budget left -------------------


def test_cloud_is_healthy_with_a_valid_token_and_budget(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-valid")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (
                200,
                {"token": {"revoked": False}, "budget": {"remaining": 120.0}},
            ),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_HEALTHY
    assert result["provider"] == "pib.Cloud"
    assert result["probe"] == health.PROBE_CLOUD_USAGE
    assert result["budgetKnown"] is True
    assert result["budgetRemaining"] == 120.0


def test_cloud_without_a_budget_ceiling_is_healthy(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-unlimited")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (
                200,
                {"token": {"revoked": False}, "budget": {"remaining": None}},
            ),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_HEALTHY
    assert result["budgetKnown"] is True
    assert result["budgetRemaining"] is None


def test_cloud_with_a_spent_budget_is_not_green(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-spent")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (
                200,
                {"token": {"revoked": False}, "budget": {"remaining": 0.0}},
            ),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_BUDGET_EXHAUSTED
    assert result["budgetRemaining"] == 0.0


def test_cloud_with_a_rejected_token_is_not_green(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-wrong")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (401, {"error": {"code": "invalid_token"}}),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_TOKEN_INVALID
    assert result["status"] == 401


def test_cloud_with_a_revoked_token_is_not_green(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-revoked")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (
                200,
                {"token": {"revoked": True}, "budget": {"remaining": 10.0}},
            ),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_TOKEN_INVALID


def test_cloud_payment_required_is_budget_exhausted(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-spent")
    _responses(
        monkeypatch,
        {
            "/health": (200, {"status": "ok"}),
            "/usage": (402, {"error": {"code": "budget_exhausted"}}),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_BUDGET_EXHAUSTED


def test_cloud_down_is_unreachable_even_with_a_valid_token(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, "tok-valid")
    _responses(
        monkeypatch,
        {
            "/health": (None, None),
            "/usage": (
                200,
                {"token": {"revoked": False}, "budget": {"remaining": 5.0}},
            ),
        },
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_UNREACHABLE
    assert result["probe"] == health.PROBE_HEALTH


def test_cloud_without_a_stored_token_is_unknown_not_red(app_ctx, monkeypatch):
    _stored_secret(monkeypatch, None)
    calls = _responses(
        monkeypatch, {"/health": (200, {"status": "ok"}), "/usage": (200, {})}
    )

    result = health.probe_model(_cloud_row(app_ctx).id)

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] in (
        health.REASON_NO_KEY,
        health.REASON_KEY_STORE_LOCKED,
    )
    # No token, so the usage route is never called with an empty credential.
    assert all(url.endswith("/health") for url, _ in calls)


# --- other providers: endpoint from the catalogue ---------------------------


def test_provider_without_a_catalogue_endpoint_is_unknown_not_red(app_ctx, monkeypatch):
    model = _fake_model("Some New Provider", "mystery-model", None)
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, "some-key")

    result = health.probe_model(1)

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] == health.REASON_NO_ENDPOINT


def test_bearer_models_probe_uses_the_catalogue_endpoint(app_ctx, monkeypatch):
    call = _responses(monkeypatch, {"/models": (200, {"data": []})})
    model = _fake_model("OpenAI", "gpt-6", "https://llm.example/v1")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_HEALTHY
    assert call[0][0] == "https://llm.example/v1/models"
    assert call[0][1]["Authorization"] == "Bearer sk-test"


def test_google_probe_passes_the_key_as_a_query_parameter(app_ctx, monkeypatch):
    call = _responses(monkeypatch, {"/models?key=g-key": (200, {"models": []})})
    model = _fake_model("Google", "gemini-3.8-flash", "https://llm.example/v1beta")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, "g-key")

    result = health.probe_model(1)

    assert result["state"] == health.STATE_HEALTHY
    assert call[0][0] == "https://llm.example/v1beta/models?key=g-key"


def test_a_rejected_key_is_red_with_its_own_reason(app_ctx, monkeypatch):
    _responses(monkeypatch, {"/models": (401, {"error": "bad key"})})
    model = _fake_model("OpenAI", "gpt-6", "https://llm.example/v1")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, "sk-wrong")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_KEY_REJECTED


def test_a_provider_without_a_stored_key_is_unknown(app_ctx, monkeypatch):
    model = _fake_model("OpenAI", "gpt-6", "https://llm.example/v1")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, None)

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] in (
        health.REASON_NO_KEY,
        health.REASON_KEY_STORE_LOCKED,
    )


def test_an_unreachable_endpoint_is_red(app_ctx, monkeypatch):
    _responses(monkeypatch, {"/models": (None, None)})
    model = _fake_model("OpenAI", "gpt-6", "https://llm.example/v1")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_UNREACHABLE


# --- the on-device model and the route --------------------------------------


def test_local_model_probe_needs_no_key(monkeypatch):
    model = _fake_model("Local", "qwen-fast", "http://host.docker.internal:11434/v1")
    _serve(monkeypatch, model)
    monkeypatch.setattr(
        key_store_service, "unlocked_secret_for_api_name", lambda api_name: None
    )
    call = _responses(monkeypatch, {"/api/tags": (200, {"models": []})})

    result = health.probe_model(6)

    assert result["state"] == health.STATE_HEALTHY
    assert call[0][0] == "http://host.docker.internal:11434/api/tags"


def test_missing_row_returns_none(app_ctx):
    assert health.probe_model(999_999) is None


def test_route_answers_the_payload_and_404_for_a_missing_row(app, monkeypatch):
    monkeypatch.setattr(
        health,
        "probe_model",
        lambda model_id: {"state": "healthy"} if model_id == CLOUD_MODEL_ID else None,
    )
    client = app.test_client()

    ok = client.get(f"/provider/{CLOUD_MODEL_ID}/health")
    missing = client.get("/provider/999999/health")

    assert ok.status_code == 200
    assert ok.get_json()["state"] == "healthy"
    assert missing.status_code == 404


class _FakeModel:
    """A catalogue row without a database."""

    def __init__(self, provider, api_name="mystery-model", visual_name="Mystery"):
        self.provider = provider
        self.api_name = api_name
        self.visual_name = visual_name
