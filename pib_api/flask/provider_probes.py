"""How each catalogue provider is reached, authenticated and status-checked.

Data, not logic. ``provider_health_service`` reads this and never names a host
or an auth header itself, because a production module may not carry strings
that look like catalogue model ids (``test_production_models_are_in_the_catalogue``
exempts this module as a whole for that reason: a provider's API host and its
HTTP auth header are account wiring, not a catalogue row).

Three facts live here per provider:

``kind``
    How the probe reads the account. ``cloud`` uses pib.Cloud's own liveness and
    usage routes (the token proves itself and carries the budget), ``tags`` is
    the on-device model's tag listing on its configured endpoint, ``models`` is
    an account's model listing, which proves the host answer *and* the key.

``auth``
    How the stored secret is presented: ``bearer`` (``Authorization: Bearer``),
    ``query`` (a query parameter, see ``auth_param``), ``header`` (the
    provider's own header, see ``auth_header`` and ``extra_headers``), or
    ``none`` for a route that needs no credential.

``status_url`` / ``status_format``
    The vendor's public status feed, so a robot without a key can still say
    whether the provider reports a fault. ``statuspage`` reads a Statuspage v2
    document (``status.indicator``); ``gcp_incidents`` reads Google's incident
    feed and counts an open incident whose affected products contain
    ``status_product``.

The dict is keyed by ``Provider.name``, the account name the catalogue seeds.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

STATUS_FORMAT_STATUSPAGE = "statuspage"
STATUS_FORMAT_GCP_INCIDENTS = "gcp_incidents"

KIND_CLOUD = "cloud"
KIND_MODELS = "models"
KIND_TAGS = "tags"

PROVIDER_PROBES: Dict[str, Dict[str, Any]] = {
    "Google": {
        "kind": KIND_MODELS,
        "url": "https://generativelanguage.googleapis.com/v1beta/models",
        "auth": "query",
        "auth_param": "key",
        "status_url": "https://status.cloud.google.com/incidents.json",
        "status_format": STATUS_FORMAT_GCP_INCIDENTS,
        "status_product": "Vertex Gemini API",
    },
    "OpenAI": {
        "kind": KIND_MODELS,
        "url": "https://api.openai.com/v1/models",
        "auth": "bearer",
        "status_url": "https://status.openai.com/api/v2/status.json",
        "status_format": STATUS_FORMAT_STATUSPAGE,
    },
    "Anthropic": {
        "kind": KIND_MODELS,
        "url": "https://api.anthropic.com/v1/models",
        "auth": "header",
        "auth_header": "x-api-key",
        "extra_headers": {"anthropic-version": "2023-06-01"},
        "status_url": "https://status.claude.com/api/v2/status.json",
        "status_format": STATUS_FORMAT_STATUSPAGE,
    },
    "pib.Cloud": {"kind": KIND_CLOUD},
    "Local": {"kind": KIND_TAGS},
}


def probe_for(provider_name: Optional[str]) -> Dict[str, Any]:
    """The wiring of one account; empty when this build knows nothing about it."""
    if not isinstance(provider_name, str):
        return {}
    return dict(PROVIDER_PROBES.get(provider_name.strip(), {}))


def status_source(probe: Dict[str, Any]) -> Optional[str]:
    """The vendor status feed of one probe, or None when it has none."""
    url = probe.get("status_url")
    return url if isinstance(url, str) and url.strip() else None
