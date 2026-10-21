"""PC-116: the provider wiring the health probe reads.

The health probe carries no host, no auth header and no vendor status page of
its own (``test_production_models_are_in_the_catalogue`` rejects strings that
look like catalogue model ids). ``provider_probes`` holds that wiring and each
``provider`` row stores its own copy, so these tests guard the two ways it can
go wrong silently:

* a catalogue provider with no wiring would answer ``unsupported`` on a robot
  that has a key, which looks like a broken provider rather than a missing entry;
* a stored copy that the reconcile step never writes would leave an upgraded
  installation without the wiring.
"""

from __future__ import annotations

import pytest

from app.app import db
from commands import _reconcile_model_catalogue, _refresh_probe_wiring
from model.provider_model import Provider
from provider_probes import (
    KIND_CLOUD,
    KIND_MODELS,
    KIND_TAGS,
    PROVIDER_PROBES,
    STATUS_FORMAT_GCP_INCIDENTS,
    STATUS_FORMAT_STATUSPAGE,
    probe_for,
    status_source,
)
from provider_registry import active_entries

KNOWN_KINDS = {KIND_CLOUD, KIND_MODELS, KIND_TAGS}
KNOWN_STATUS_FORMATS = {STATUS_FORMAT_STATUSPAGE, STATUS_FORMAT_GCP_INCIDENTS}


def test_every_catalogue_provider_has_wiring():
    """A provider a personality can point at must be probed, not guessed.

    ``Local`` is not in the catalogue: the on-device model arrives as a profile
    row, and its provider name is the visual name ``Local``.
    """
    names = {entry.provider for entry in active_entries()} | {"Local"}

    missing = sorted(name for name in names if not probe_for(name))

    assert missing == []


def test_wiring_declares_a_known_kind():
    for name, wiring in PROVIDER_PROBES.items():
        assert wiring.get("kind") in KNOWN_KINDS, name


def test_a_model_probe_carries_its_url_and_auth_style():
    for name, wiring in PROVIDER_PROBES.items():
        if wiring.get("kind") != KIND_MODELS:
            continue
        assert isinstance(wiring.get("url"), str) and wiring["url"].startswith(
            "https://"
        ), name
        assert wiring.get("auth") in {"bearer", "query", "header", "none"}, name
        if wiring.get("auth") == "query":
            assert wiring.get("auth_param"), name
        if wiring.get("auth") == "header":
            assert wiring.get("auth_header"), name


def test_a_vendor_status_feed_declares_its_format():
    for name, wiring in PROVIDER_PROBES.items():
        url = status_source(wiring)
        if url is None:
            continue
        assert url.startswith("https://"), name
        assert wiring.get("status_format") in KNOWN_STATUS_FORMATS, name
        if wiring.get("status_format") == STATUS_FORMAT_GCP_INCIDENTS:
            # Google's incident feed is a flat list; without a product name an
            # incident cannot be attributed to this provider.
            assert wiring.get("status_product"), name


def test_the_cloud_and_the_on_device_model_need_no_wiring_of_their_own():
    """Both are reached through what the installation already configured."""
    assert PROVIDER_PROBES["pib.Cloud"]["kind"] == KIND_CLOUD
    assert PROVIDER_PROBES["Local"]["kind"] == KIND_TAGS
    assert "url" not in PROVIDER_PROBES["Local"]


@pytest.mark.parametrize("name", ["", None, "Some New Provider", "  "])
def test_an_unknown_account_has_no_wiring(name):
    assert probe_for(name) == {}


def test_the_reconcile_step_writes_the_wiring_to_every_known_row(app_ctx):
    _reconcile_model_catalogue()
    db.session.flush()

    rows = Provider.query.all()

    assert rows, "the catalogue seed created no provider"
    for provider in rows:
        expected = probe_for(provider.name)
        if expected:
            assert provider.probe == expected, provider.name


def test_the_reconcile_step_leaves_an_unknown_account_alone(app_ctx):
    """A provider this build does not know keeps its stored wiring."""
    stored = {"kind": KIND_MODELS, "url": "https://gateway.internal/v1/models"}
    provider = Provider(
        name="Some New Provider",
        capabilities={},
        endpoint_base=None,
        credential_ref=None,
        probe=stored,
    )
    db.session.add(provider)
    db.session.flush()

    _refresh_probe_wiring(provider)

    assert provider.probe == stored
