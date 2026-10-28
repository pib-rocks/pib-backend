"""MCP ``get_motion_model`` uses the live REST motion-model endpoint."""

from __future__ import annotations

import asyncio
import json

import pytest

from pib_mcp_server.backend import BackendError, PibBackend
from pib_mcp_server.server import create_server


def _call(server, name: str, arguments: dict) -> dict:
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    content = getattr(result, "content", None)
    if content:
        return json.loads(content[0].text)
    return result


def test_get_motion_model_fetches_from_api():
    backend = PibBackend()
    api_payload = {
        "schemaVersion": 1,
        "chains": {"head": []},
        "postures": {},
        "prompt": "Body (centidegrees: ...)",
    }
    calls: list[tuple[str, str]] = []

    def fake_http(method, path, payload=None):
        calls.append((method, path))
        assert payload is None
        return api_payload

    backend._http = fake_http
    assert backend.get_motion_model(detail=True) == api_payload
    assert backend.get_motion_model() == {
        "schemaVersion": 1,
        "prompt": "Body (centidegrees: ...)",
    }
    assert calls == [("GET", "/system/motion-model"), ("GET", "/system/motion-model")]


def test_get_motion_model_tool_returns_compact_payload_by_default():
    backend = PibBackend()
    api_payload = {
        "schemaVersion": 1,
        "prompt": "live model",
        "chains": {"head": {"joints": []}},
    }
    backend._http = lambda method, path, payload=None: api_payload
    server = create_server(backend, actuating_enabled=False)
    response = _call(server, "get_motion_model", {})
    assert response["ok"] is True
    assert response["result"] == {"schemaVersion": 1, "prompt": "live model"}


def test_get_motion_model_detail_returns_api_payload():
    backend = PibBackend()
    api_payload = {"schemaVersion": 1, "prompt": "live model", "chains": {}}
    backend._http = lambda method, path, payload=None: api_payload
    server = create_server(backend, actuating_enabled=False)
    response = _call(server, "get_motion_model", {"detail": True})
    assert response["ok"] is True
    assert response["result"] == api_payload


def test_get_motion_model_tool_surfaces_backend_error():
    backend = PibBackend()

    def fail_http(method, path, payload=None):
        raise BackendError("api_error", "pib API returned HTTP 503", {"status": 503})

    backend._http = fail_http
    server = create_server(backend, actuating_enabled=False)
    response = _call(server, "get_motion_model", {})
    assert response["ok"] is False
    assert response["error"]["code"] == "api_error"
    assert "503" in response["error"]["message"]


def test_soul_section_advertises_the_tool_with_its_exact_name():
    from pib_hermes_config import MCP_TOOLS, MCP_TOOLS_SOUL_SECTION, mcp_tool_name

    assert "get_motion_model" in {tool for tool, _ in MCP_TOOLS}
    assert f"### {mcp_tool_name('get_motion_model')}" in MCP_TOOLS_SOUL_SECTION
