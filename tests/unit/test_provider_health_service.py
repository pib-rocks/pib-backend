"""PC-115/116: the provider health probe behind the voice-assistant status strip.

Behaviours that carry the requirement:

* pib.Cloud is green only when the service answers, the stored SmartConnect
  token is accepted and the token still has budget. A live ``/health`` alone
  is not enough.
* A probe that cannot run is ``unknown`` (neutral in the UI), never
  ``unreachable``: a missing key or a provider this build knows no wiring for
  must not look like an outage.
* The vendor status feed is a second, keyless stage. It explains a partial
  outage and relabels a failed account probe as ``provider_outage``, but it
  never turns a listing that answered into a failure.
"""

from __future__ import annotations

import pytest

from model.provider_model import Provider, RegistryModel
from provider_probes import PROVIDER_PROBES, probe_for
from service import key_store_service, provider_health_service as health

CLOUD_MODEL_ID = 5

STATUSPAGE_FEED = "https://status.example/api/v2/status.json"
GCP_FEED = "https://status.example/incidents.json"


def _cloud_row(app_ctx) -> RegistryModel:
    return RegistryModel.query.filter_by(api_name="pib-cloud").one()


def _stored_secret(monkeypatch, value):
    """Pretend the key store is unlocked and holds this secret for the row."""
    monkeypatch.setattr(
        key_store_service, "unlocked_secret_for_api_name", lambda api_name: value
    )


def _responses(monkeypatch, table):
    """Replace the HTTP layer. ``table`` maps a URL suffix to (status, body).

    The built-in wiring carries a vendor status feed for each account, and the
    probe reads it beside the account route. Every feed therefore answers
    "operational" unless a test names it explicitly, so a test about the account
    route does not have to spell the feed out.
    """
    calls = []

    def default_status_body(wiring):
        if wiring.get("status_format") == "gcp_incidents":
            return []
        return {"status": {"indicator": "none", "description": "Operational"}}

    def fake_get(url, headers):
        calls.append((url, headers))
        for suffix, (status, body) in table.items():
            if url.endswith(suffix):
                error = "" if status is not None else "URLError: down"
                return status, body, error, 7
        for wiring in PROVIDER_PROBES.values():
            feed = wiring.get("status_url")
            if isinstance(feed, str) and url == feed:
                return 200, default_status_body(wiring), "", 7
        raise AssertionError(f"unexpected probe URL: {url}")

    monkeypatch.setattr(health, "_http_get", fake_get)
    return calls


def _fake_model(provider_name, api_name, endpoint_base=None, probe=None):
    """A catalogue row without a database."""
    provider = Provider(
        name=provider_name, capabilities={}, endpoint_base=endpoint_base
    )
    provider.probe = probe
    return _FakeModel(
        provider,
        api_name=api_name,
        visual_name=f"{provider_name} {api_name}",
    )


def _serve(monkeypatch, model):
    monkeypatch.setattr(health.provider_service, "get_model_by_id", lambda _id: model)


def _statuspage(indicator: str, description: str = "") -> dict:
    return {"status": {"indicator": indicator, "description": description}}


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


# --- third-party accounts: wiring from provider_probes ----------------------


def test_openai_probe_reads_its_wiring_and_sends_a_bearer_token(monkeypatch):
    call = _responses(monkeypatch, {"/models": (200, {"data": []})})
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6"))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_HEALTHY
    assert result["probe"] == health.PROBE_MODELS
    account_calls = [entry for entry in call if entry[0] == probe_for("OpenAI")["url"]]
    assert account_calls[0][1]["Authorization"] == "Bearer sk-test"


def test_anthropic_probe_sends_its_own_auth_header(monkeypatch):
    call = _responses(monkeypatch, {"/models": (200, {"data": []})})
    _serve(monkeypatch, _fake_model("Anthropic", "claude-sonnet-5-5"))
    _stored_secret(monkeypatch, "sk-ant")

    result = health.probe_model(4)

    assert result["state"] == health.STATE_HEALTHY
    wiring = probe_for("Anthropic")
    account_calls = [entry for entry in call if entry[0] == wiring["url"]]
    assert account_calls[0][1][wiring["auth_header"]] == "sk-ant"
    for name, value in wiring["extra_headers"].items():
        assert account_calls[0][1][name] == value


def test_google_probe_passes_the_key_as_a_query_parameter(monkeypatch):
    wiring = probe_for("Google")
    call = _responses(
        monkeypatch, {f"?{wiring['auth_param']}=g-key": (200, {"models": []})}
    )
    _serve(monkeypatch, _fake_model("Google", "gemini-3.8-flash"))
    _stored_secret(monkeypatch, "g-key")

    result = health.probe_model(1)

    assert result["state"] == health.STATE_HEALTHY
    assert [url for url, _ in call if url.startswith(wiring["url"])] == [
        f"{wiring['url']}?key=g-key"
    ]


def test_a_rejected_key_is_red_with_its_own_reason(monkeypatch):
    _responses(monkeypatch, {"/models": (401, {"error": "bad key"})})
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6"))
    _stored_secret(monkeypatch, "sk-wrong")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_KEY_REJECTED


def test_a_provider_without_a_stored_key_is_unknown_and_not_probed(monkeypatch):
    call = _responses(monkeypatch, {"/models": (200, {"data": []})})
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6"))
    _stored_secret(monkeypatch, None)

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] in (
        health.REASON_NO_KEY,
        health.REASON_KEY_STORE_LOCKED,
    )
    # Without a key only the vendor feed is read, never the account route.
    assert [url for url, _ in call] == [probe_for("OpenAI")["status_url"]]


def test_an_unreachable_endpoint_is_red(monkeypatch):
    _responses(monkeypatch, {"/models": (None, None)})
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6"))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_UNREACHABLE


def test_a_provider_this_build_does_not_know_is_unknown(monkeypatch):
    _serve(monkeypatch, _fake_model("Some New Provider", "mystery-model"))
    _stored_secret(monkeypatch, "some-key")

    result = health.probe_model(1)

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] == health.REASON_UNSUPPORTED


def test_a_stored_wiring_wins_over_the_built_in_catalogue(monkeypatch):
    """An installation can point an account at its own gateway."""
    call = _responses(monkeypatch, {"/models": (200, {"data": []})})
    wiring = {
        "kind": "models",
        "url": "https://gateway.internal/v1/models",
        "auth": "bearer",
    }
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, "sk-test")

    health.probe_model(3)

    assert call[0][0] == "https://gateway.internal/v1/models"


# --- vendor status feed (PC-116, second stage) ------------------------------


def _status_wiring(feed: str, fmt: str, product: str | None = None) -> dict:
    wiring = {
        "kind": "models",
        "url": "https://llm.example/v1/models",
        "auth": "bearer",
    }
    wiring["status_url"] = feed
    wiring["status_format"] = fmt
    if product:
        wiring["status_product"] = product
    return wiring


def test_a_healthy_account_reports_the_vendor_as_operational(monkeypatch):
    wiring = _status_wiring(STATUSPAGE_FEED, "statuspage")
    _responses(
        monkeypatch,
        {
            "/status.json": (200, _statuspage("none", "All Systems Operational")),
            "/models": (200, {"data": []}),
        },
    )
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_HEALTHY
    assert result["vendorStatus"] == health.VENDOR_OPERATIONAL


def test_a_vendor_outage_relabels_a_failed_probe(monkeypatch):
    wiring = _status_wiring(STATUSPAGE_FEED, "statuspage")
    _responses(
        monkeypatch,
        {
            "/status.json": (200, _statuspage("major", "Elevated error rates")),
            "/models": (503, {"error": "unavailable"}),
        },
    )
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_UNREACHABLE
    assert result["reason"] == health.REASON_PROVIDER_OUTAGE
    assert result["vendorStatus"] == health.VENDOR_DEGRADED
    assert "Elevated error rates" in result["vendorDetail"]


def test_a_vendor_outage_does_not_turn_a_working_key_red(monkeypatch):
    wiring = _status_wiring(STATUSPAGE_FEED, "statuspage")
    _responses(
        monkeypatch,
        {
            "/status.json": (200, _statuspage("minor", "Partial outage")),
            "/models": (200, {"data": []}),
        },
    )
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_HEALTHY
    assert result["vendorStatus"] == health.VENDOR_DEGRADED


def test_the_status_feed_is_read_even_without_a_key(monkeypatch):
    wiring = _status_wiring(STATUSPAGE_FEED, "statuspage")
    calls = _responses(monkeypatch, {"/status.json": (200, _statuspage("major"))})
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, None)

    result = health.probe_model(3)

    # Still neutral - this personality cannot answer without a key - but the
    # vendor fault is reported beside it.
    assert result["state"] == health.STATE_UNKNOWN
    assert result["vendorStatus"] == health.VENDOR_DEGRADED
    assert [url for url, _ in calls] == [STATUSPAGE_FEED]


def test_an_unreadable_status_feed_stays_unknown(monkeypatch):
    wiring = _status_wiring(STATUSPAGE_FEED, "statuspage")
    _responses(
        monkeypatch,
        {"/status.json": (None, None), "/models": (200, {"data": []})},
    )
    _serve(monkeypatch, _fake_model("OpenAI", "gpt-6", probe=wiring))
    _stored_secret(monkeypatch, "sk-test")

    result = health.probe_model(3)

    assert result["state"] == health.STATE_HEALTHY
    assert result["vendorStatus"] == health.VENDOR_UNKNOWN


def test_a_google_incident_on_the_product_is_a_vendor_fault(monkeypatch):
    wiring = _status_wiring(GCP_FEED, "gcp_incidents", "Vertex Gemini API")
    feed = [
        {
            "end": None,
            "affected_products": [{"title": "Vertex Gemini API"}],
            "external_desc": "Gemini API serving errors",
        },
        {
            "end": "2026-10-20T14:00:00+00:00",
            "affected_products": [{"title": "Vertex Gemini API"}],
            "external_desc": "closed incident",
        },
    ]
    _responses(monkeypatch, {"/incidents.json": (200, feed)})
    _serve(monkeypatch, _fake_model("Google", "gemini-3.8-flash", probe=wiring))
    _stored_secret(monkeypatch, None)

    result = health.probe_model(1)

    assert result["vendorStatus"] == health.VENDOR_DEGRADED
    assert "serving errors" in result["vendorDetail"]


def test_a_google_incident_on_another_product_is_not_this_providers_fault(
    monkeypatch,
):
    wiring = _status_wiring(GCP_FEED, "gcp_incidents", "Vertex Gemini API")
    feed = [
        {
            "end": None,
            "affected_products": [{"title": "Cloud Storage"}],
            "external_desc": "Bucket latency",
        }
    ]
    _responses(monkeypatch, {"/incidents.json": (200, feed)})
    _serve(monkeypatch, _fake_model("Google", "gemini-3.8-flash", probe=wiring))
    _stored_secret(monkeypatch, None)

    result = health.probe_model(1)

    assert result["vendorStatus"] == health.VENDOR_OPERATIONAL


def test_a_closed_google_incident_is_operational(monkeypatch):
    wiring = _status_wiring(GCP_FEED, "gcp_incidents", "Vertex Gemini API")
    feed = [
        {
            "end": "2026-10-20T14:00:00+00:00",
            "affected_products": [{"title": "Vertex Gemini API"}],
        }
    ]
    _responses(monkeypatch, {"/incidents.json": (200, feed)})
    _serve(monkeypatch, _fake_model("Google", "gemini-3.8-flash", probe=wiring))
    _stored_secret(monkeypatch, None)

    result = health.probe_model(1)

    assert result["vendorStatus"] == health.VENDOR_OPERATIONAL


# --- the on-device model and the route --------------------------------------


def test_local_model_probe_needs_no_key(monkeypatch):
    model = _fake_model("Local", "qwen-fast", "http://host.docker.internal:11434/v1")
    _serve(monkeypatch, model)
    _stored_secret(monkeypatch, None)
    call = _responses(monkeypatch, {"/api/tags": (200, {"models": []})})

    result = health.probe_model(6)

    assert result["state"] == health.STATE_HEALTHY
    assert result["probe"] == health.PROBE_TAGS
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
