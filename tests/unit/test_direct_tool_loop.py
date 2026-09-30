"""Direct tool loop: MCP calls continue until a final answer, or not at all."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from voice_assistant.direct_tool_loop import (  # noqa: E402
    IMAGE_TOOL,
    PINNED_CHECKED_ON,
    PINNED_ENDPOINT,
    PINNED_MODEL,
    PINNED_PROVIDER,
    DirectToolLoopError,
    assert_stable_url,
    build_gemini_request,
    gemini_complete,
    image_tool_allowed,
    parse_gemini_response,
    run_direct_turn,
    supports_tool_endpoint,
    tool_calling_enabled,
    tools_for_turn,
)


def test_mcp_declarations_include_the_image_tool():
    from voice_assistant.direct_tool_loop import mcp_tool_declarations

    names = {item["name"] for item in mcp_tool_declarations()}
    assert IMAGE_TOOL in names
    assert "list_poses" in names
    image = next(item for item in mcp_tool_declarations() if item["name"] == IMAGE_TOOL)
    assert image["parameters"]["type"] == "object"


def test_pinned_endpoint_is_the_stable_generate_content_method():
    assert PINNED_PROVIDER == "gemini"
    assert PINNED_MODEL == "gemini-3.5-flash"
    assert PINNED_CHECKED_ON == "2026-09-30"
    assert PINNED_ENDPOINT.endswith(f"/v1/models/{PINNED_MODEL}:generateContent")
    assert "beta" not in PINNED_ENDPOINT
    assert supports_tool_endpoint("gemini-3.5-flash") is True
    assert supports_tool_endpoint("hermes-agent") is True
    assert supports_tool_endpoint("gpt-4o") is False
    with pytest.raises(DirectToolLoopError, match="beta"):
        assert_stable_url(
            "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions"
        )


def test_tool_switch_defaults_on_and_an_explicit_false_wins():
    personality = SimpleNamespace()
    assert tool_calling_enabled(personality) is True
    personality.tool_calling = False
    assert tool_calling_enabled(personality) is False
    personality.tool_calling = True
    assert tool_calling_enabled(personality) is True


def test_image_tool_requires_the_switch_and_the_capability():
    personality = MagicMock()
    personality.assistant_model.has_image_support = True
    personality.assistant_model.capabilities = {"images": True}
    assert image_tool_allowed(personality, True) is True
    assert image_tool_allowed(personality, False) is False
    personality.assistant_model.capabilities = {"images": False}
    assert image_tool_allowed(personality, True) is False


def test_tools_off_drops_every_tool_including_the_image_tool():
    declarations = [
        {"name": "list_poses", "description": "poses", "parameters": {}},
        {"name": IMAGE_TOOL, "description": "camera", "parameters": {}},
    ]
    assert tools_for_turn(declarations, tool_calling=False, allow_image=True) == []
    names = [
        item["name"]
        for item in tools_for_turn(declarations, tool_calling=True, allow_image=False)
    ]
    assert names == ["list_poses"]


def test_loop_executes_a_tool_and_continues_until_the_final_answer():
    seen = []

    def complete(messages, tools):
        seen.append((messages, tools))
        user = [item for item in messages if item["role"] == "user"][-1]
        assert user["content"] == "Welche Posen gibt es?"
        assert "image" not in json.dumps(user)
        if len(seen) == 1:
            return {
                "text": "",
                "tool_calls": [
                    {"id": "list_poses-0", "name": "list_poses", "arguments": {}}
                ],
                "raw_parts": [{"functionCall": {"name": "list_poses", "args": {}}}],
            }
        assert messages[-1]["role"] == "tool"
        assert messages[-1]["content"] == {"ok": True, "result": ["Rest"]}
        return {"text": "Die Pose heisst Rest.", "tool_calls": []}

    calls = []

    def execute(name, arguments):
        calls.append((name, arguments))
        return {"ok": True, "result": ["Rest"]}

    answer = "".join(
        run_direct_turn(
            system_prompt="Du bist pib.",
            user_text="Welche Posen gibt es?",
            history=[("Hallo", True), ("Welche Posen gibt es?", True)],
            tool_calling=True,
            allow_image=True,
            declarations=[
                {"name": "list_poses", "description": "poses"},
                {"name": IMAGE_TOOL, "description": "camera"},
            ],
            complete=complete,
            execute_tool=execute,
        )
    )

    assert answer == "Die Pose heisst Rest."
    assert calls == [("list_poses", {})]
    offered = [item["name"] for item in seen[0][1]]
    assert offered == ["list_poses", IMAGE_TOOL]
    # The stored copy of the current user text is not sent twice.
    user_texts = [item["content"] for item in seen[0][0] if item["role"] == "user"]
    assert user_texts == ["Hallo", "Welche Posen gibt es?"]


def test_tool_calling_off_makes_one_request_without_tools_or_an_image():
    def execute(name, arguments):
        raise AssertionError(f"tool {name} must not run")

    def complete(messages, tools):
        assert tools is None
        encoded = json.dumps(messages)
        assert "inlineData" not in encoded
        assert IMAGE_TOOL not in encoded
        assert messages[-1] == {"role": "user", "content": "Nur Text."}
        return {"text": "Nur Text zurueck.", "tool_calls": []}

    answer = "".join(
        run_direct_turn(
            system_prompt="Du bist pib.",
            user_text="Nur Text.",
            history=[],
            tool_calling=False,
            allow_image=True,
            declarations=[{"name": IMAGE_TOOL, "description": "camera"}],
            complete=complete,
            execute_tool=execute,
        )
    )
    assert answer == "Nur Text zurueck."


def test_loop_stops_when_the_model_never_answers():
    def complete(messages, tools):
        return {
            "text": "",
            "tool_calls": [
                {"id": "list_poses-0", "name": "list_poses", "arguments": {}}
            ],
        }

    with pytest.raises(DirectToolLoopError, match="final answer"):
        list(
            run_direct_turn(
                system_prompt="Du bist pib.",
                user_text="Hi",
                history=[],
                tool_calling=True,
                allow_image=False,
                declarations=[{"name": "list_poses", "description": "poses"}],
                complete=complete,
                execute_tool=lambda name, arguments: {"ok": True},
                max_rounds=2,
            )
        )


def test_gemini_request_uses_the_pinned_url_and_omits_tools_when_absent(monkeypatch):
    captured = {}

    class Response:
        def read(self):
            return json.dumps(
                {"candidates": [{"content": {"parts": [{"text": "pong"}]}}]}
            ).encode()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def fake_urlopen(request, timeout=None):
        captured["url"] = request.full_url
        captured["header"] = request.get_header("X-goog-api-key")
        captured["body"] = json.loads(request.data.decode())
        return Response()

    monkeypatch.setattr("voice_assistant.direct_tool_loop.urlopen", fake_urlopen)
    monkeypatch.setenv("GOOGLE_API_KEY", "test-key")
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)

    result = gemini_complete(
        [
            {"role": "system", "content": "Du bist pib."},
            {"role": "user", "content": "ping"},
        ],
        None,
    )

    assert result["text"] == "pong"
    assert result["tool_calls"] == []
    assert captured["url"] == PINNED_ENDPOINT
    assert "beta" not in captured["url"]
    assert "key=" not in captured["url"]
    assert captured["header"] == "test-key"
    assert "tools" not in captured["body"]
    assert captured["body"]["contents"] == [
        {"role": "user", "parts": [{"text": "ping"}]}
    ]
    encoded = json.dumps(captured["body"])
    assert "inlineData" not in encoded
    assert "image" not in encoded


def test_gemini_request_declares_tools_and_keeps_the_function_call_parts():
    url, headers, body = build_gemini_request(
        [
            {"role": "system", "content": "Du bist pib."},
            {"role": "user", "content": "Schau hin."},
            {
                "role": "assistant",
                "content": "",
                "raw_parts": [
                    {
                        "functionCall": {"name": "list_poses", "args": {}},
                        "thoughtSignature": "sig",
                    }
                ],
            },
            {
                "role": "tool",
                "name": "list_poses",
                "content": {"ok": True, "result": []},
            },
        ],
        [{"name": "list_poses", "description": "poses", "parameters": {}}],
        "test-key",
    )
    assert url == PINNED_ENDPOINT
    assert headers["x-goog-api-key"] == "test-key"
    assert "beta" not in url
    names = [item["name"] for item in body["tools"][0]["functionDeclarations"]]
    assert names == ["list_poses"]
    model_turn = body["contents"][1]
    assert model_turn["role"] == "model"
    assert model_turn["parts"][0]["thoughtSignature"] == "sig"
    assert body["contents"][2]["parts"][0]["functionResponse"]["name"] == "list_poses"


def test_parse_gemini_response_reads_a_function_call_and_skips_thoughts():
    parsed = parse_gemini_response(
        {
            "candidates": [
                {
                    "content": {
                        "parts": [
                            {"text": "thinking", "thought": True},
                            {"functionCall": {"name": "list_poses", "args": {}}},
                        ]
                    }
                }
            ]
        }
    )
    assert parsed["text"] == ""
    assert parsed["tool_calls"] == [
        {"id": "list_poses-1", "name": "list_poses", "arguments": {}}
    ]
    assert parsed["raw_parts"][1]["functionCall"]["name"] == "list_poses"


def test_personality_client_reads_the_tool_switch(monkeypatch):
    from pib_api_client.voice_assistant_client import Personality

    model = MagicMock()
    model.api_name = "gemini-3.5-flash"
    monkeypatch.setattr(
        "pib_api_client.voice_assistant_client.get_default_provider",
        lambda: (True, model),
    )
    payload = {
        "gender": "Female",
        "pauseThreshold": 0.8,
        "messageHistory": 5,
        "description": "soul text",
        "providerRef": "default",
        "channel": "direct",
        "effectiveChannel": "direct",
    }
    missing = Personality(payload)
    assert missing.tool_calling is True
    off = Personality({**payload, "toolCalling": False})
    assert off.tool_calling is False


def test_personality_api_defaults_tool_calling_on_and_can_switch_it_off(app, app_ctx):
    from model.assistant_model import AssistantModel

    client = app.test_client()
    model = AssistantModel.query.first()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "ToolBot",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "assistantModelId": model.id,
            "description": "Ein Text.",
            "channel": "direct",
        },
    )
    assert created.status_code == 201
    body = created.get_json()
    assert body["toolCalling"] is True

    switched = client.put(
        f"/voice-assistant/personality/{body['personalityId']}",
        json={"toolCalling": False},
    )
    assert switched.status_code == 200
    assert switched.get_json()["toolCalling"] is False
    assert switched.get_json()["description"] == body["description"]

    restored_channel = client.put(
        f"/voice-assistant/personality/{body['personalityId']}",
        json={"channel": "direct"},
    )
    assert restored_channel.get_json()["toolCalling"] is False
