"""Capability manifest loading, MCP enforcement, and HTTP exposure."""

from __future__ import annotations

import pytest

from pib_capability.manifest import (
    AVAILABLE_VARIANTS,
    UnknownVariantError,
    load_manifest,
    remove_tools_not_in_manifest,
    skill_names,
)
from pib_mcp_server.server import create_server
from test_pib_mcp_server import FakeBackend, _tools


@pytest.mark.parametrize("variant", AVAILABLE_VARIANTS)
def test_manifest_loads_and_validates_for_each_variant(variant: str):
    doc = load_manifest(variant)
    assert doc["skills"]
    assert doc["learned_skills"] == []


def test_unknown_variant_raises_typed_error():
    with pytest.raises(UnknownVariantError) as error:
        load_manifest("not-a-variant")
    assert error.value.available == AVAILABLE_VARIANTS
    assert "not-a-variant" in str(error.value)


def test_filtering_removes_absent_tools_and_keeps_the_rest():
    backend = FakeBackend()
    server = create_server(backend, actuating_enabled=True)
    manifest = load_manifest("pib5edu")
    trimmed = {**manifest, "skills": [name for name in manifest["skills"] if name != "soul_append"]}
    remove_tools_not_in_manifest(server, trimmed)
    names = {tool.name for tool in _tools(server)}
    assert "soul_append" not in names
    assert "list_motors" in names
    assert len(names) == len(trimmed["skills"])


def test_mcp_server_exposes_exactly_manifest_skills(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("PIB_ROBOT_VARIANT", "pib5advanced")
    monkeypatch.delenv("PIB_CAPABILITY_ENFORCE", raising=False)
    manifest = load_manifest("pib5advanced")
    server = create_server(FakeBackend(), actuating_enabled=True)
    exposed = {tool.name for tool in _tools(server)}
    assert exposed == set(skill_names(manifest))


def test_capabilities_endpoint_matches_load_manifest(app, monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("PIB_ROBOT_VARIANT", "pib4edu")
    expected = load_manifest("pib4edu")
    with app.test_client() as client:
        response = client.get("/api/v1/capabilities")
    assert response.status_code == 200
    assert response.get_json() == expected


def test_pib5edu_and_pib5advanced_differ_only_in_hardware_fields():
    edu = load_manifest("pib5edu")
    advanced = load_manifest("pib5advanced")
    edu_hardware = {key: edu[key] for key in ("tier", "platform", "sensors", "limits")}
    advanced_hardware = {
        key: advanced[key] for key in ("tier", "platform", "sensors", "limits")
    }
    assert edu_hardware != advanced_hardware
    for key in ("skills", "learned_skills", "cloud", "degraded_without_cloud"):
        assert edu[key] == advanced[key]
