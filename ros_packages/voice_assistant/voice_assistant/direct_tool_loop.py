"""Direct chat tool loop.

A Direct turn asks the model, runs MCP tool calls, and continues until the
model returns a final answer. The per-personality switch turns that off: the
request then carries no tools and no image. A camera frame is only available
as the result of the capture_image tool.

Provider and model were read from this account on 2026-09-30.
``GET https://generativelanguage.googleapis.com/v1/models`` lists
``gemini-3.5-flash``. The stable method on that host is
``/v1/models/gemini-3.5-flash:generateContent``, and a probe of it returned a
function call. ``/v1/openai/chat/completions`` returned 404. The
OpenAI-compatible shim under ``/v1beta`` is not used.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Callable, Iterator, Mapping, Optional, Sequence
from urllib.request import Request, urlopen

logger = logging.getLogger(__name__)

PINNED_PROVIDER = "gemini"
PINNED_MODEL = "gemini-3.5-flash"
PINNED_CHECKED_ON = "2026-09-30"
PINNED_ENDPOINT = (
    "https://generativelanguage.googleapis.com/v1/models/"
    "gemini-3.5-flash:generateContent"
)
# The Hermes row is not itself a generateContent model. Its chat pin is the
# same model the Hermes profile already writes.
GEMINI_API_NAMES = frozenset({PINNED_MODEL, "hermes-agent"})
IMAGE_TOOL = "capture_image"
MAX_TOOL_ROUNDS = 8

Complete = Callable[[list[dict[str, Any]], Optional[list[dict[str, Any]]]], dict]
ExecuteTool = Callable[[str, dict[str, Any]], Any]


class DirectToolLoopError(Exception):
    """The Direct turn stopped before a final answer."""


def assert_stable_url(url: str) -> str:
    """Refuse any endpoint whose path is a beta surface."""
    if not isinstance(url, str) or not url.strip():
        raise DirectToolLoopError("refusing an empty endpoint")
    if "beta" in url.lower():
        raise DirectToolLoopError("refusing a beta endpoint")
    return url


def supports_tool_endpoint(api_name: Optional[str]) -> bool:
    """True when this registry name resolves to the pinned non-beta model."""
    return api_name in GEMINI_API_NAMES


def tool_calling_enabled(personality: object) -> bool:
    """Per-personality switch. Missing or non-boolean values stay on."""
    raw = getattr(personality, "tool_calling", True)
    if isinstance(raw, bool):
        return raw
    return True


def image_tool_allowed(personality: object, tool_calling: bool) -> bool:
    """Images need the tool switch and a model that can take them.

    The frame is never attached to the user message. This only decides
    whether capture_image is offered.
    """
    if not tool_calling:
        return False
    model = getattr(personality, "assistant_model", None)
    if model is None:
        return False
    capabilities = getattr(model, "capabilities", None)
    if isinstance(capabilities, dict) and "images" in capabilities:
        return bool(capabilities.get("images"))
    flag = getattr(model, "has_image_support", False)
    return flag if isinstance(flag, bool) else False


def tools_for_turn(
    declarations: Sequence[Mapping[str, Any]],
    *,
    tool_calling: bool,
    allow_image: bool,
) -> list[dict[str, Any]]:
    """Tools actually sent. Off means none, so capture_image cannot run."""
    if not tool_calling:
        return []
    offered: list[dict[str, Any]] = []
    for item in declarations:
        name = item.get("name")
        if not isinstance(name, str) or not name:
            continue
        if name == IMAGE_TOOL and not allow_image:
            continue
        offered.append(dict(item))
    return offered


def gemini_api_key() -> str:
    """Same names the voice container already receives from password.env."""
    return os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY") or ""


def _history_without_current_user(
    history: Sequence[tuple[str, bool]], user_text: str
) -> list[tuple[str, bool]]:
    """Drop a trailing copy of the message this turn just stored."""
    items = list(history)
    if items and items[-1][1] is True and items[-1][0] == user_text:
        return items[:-1]
    return items


def run_direct_turn(
    *,
    system_prompt: str,
    user_text: str,
    history: Sequence[tuple[str, bool]],
    tool_calling: bool,
    allow_image: bool,
    declarations: Optional[Sequence[Mapping[str, Any]]] = None,
    complete: Optional[Complete] = None,
    execute_tool: Optional[ExecuteTool] = None,
    max_rounds: int = MAX_TOOL_ROUNDS,
) -> Iterator[str]:
    """Yield the model's final answer. Tool results are not spoken.

    ``history`` entries are ``(content, is_user)``. The user message is text
    only: nothing in this function attaches an image.
    """
    if not tool_calling:
        allow_image = False
        declarations = []
    elif declarations is None:
        declarations = mcp_tool_declarations()

    offered = tools_for_turn(
        declarations or [],
        tool_calling=tool_calling,
        allow_image=allow_image,
    )
    tools = offered or None
    caller = complete or gemini_complete
    runner = execute_tool or execute_mcp_tool

    messages: list[dict[str, Any]] = [
        {"role": "system", "content": system_prompt},
    ]
    for content, is_user in _history_without_current_user(history, user_text):
        messages.append(
            {
                "role": "user" if is_user else "assistant",
                "content": content if isinstance(content, str) else str(content),
            }
        )
    messages.append({"role": "user", "content": user_text})

    for _round in range(max_rounds):
        result = caller(messages, tools if tool_calling else None)
        calls = result.get("tool_calls") or []
        if not tool_calling or not calls:
            yield result.get("text") or ""
            return
        messages.append(
            {
                "role": "assistant",
                "content": result.get("text") or "",
                "tool_calls": calls,
                "raw_parts": result.get("raw_parts"),
            }
        )
        for call in calls:
            name = call.get("name")
            if not isinstance(name, str) or not name:
                raise DirectToolLoopError("model returned a tool call without a name")
            if name == IMAGE_TOOL and not allow_image:
                raise DirectToolLoopError("image tool is not available on this turn")
            arguments = call.get("arguments") or {}
            if not isinstance(arguments, dict):
                arguments = {}
            logger.info("direct tool call name=%s", name)
            output = runner(name, arguments)
            messages.append(
                {
                    "role": "tool",
                    "name": name,
                    "tool_call_id": call.get("id") or name,
                    "content": _json_object(output),
                }
            )
    raise DirectToolLoopError("tool loop ended without a final answer")


def _json_object(value: Any) -> dict[str, Any]:
    """Gemini function responses have to be objects."""
    if isinstance(value, dict):
        return value
    return {"result": value}


def build_gemini_request(
    messages: Sequence[Mapping[str, Any]],
    tools: Optional[Sequence[Mapping[str, Any]]],
    api_key: str,
) -> tuple[str, dict[str, str], dict[str, Any]]:
    """Stable generateContent request. The key travels in a header, not the URL."""
    url = assert_stable_url(PINNED_ENDPOINT)
    if not api_key:
        raise DirectToolLoopError("GOOGLE_API_KEY is not set")
    headers = {
        "Content-Type": "application/json",
        "x-goog-api-key": api_key,
    }
    system_parts: list[str] = []
    contents: list[dict[str, Any]] = []
    pending_responses: list[dict[str, Any]] = []

    def flush_responses() -> None:
        if not pending_responses:
            return
        contents.append({"role": "user", "parts": list(pending_responses)})
        pending_responses.clear()

    for message in messages:
        role = message.get("role")
        if role == "system":
            text = message.get("content") or ""
            if text:
                system_parts.append(str(text))
            continue
        if role == "tool":
            pending_responses.append(
                {
                    "functionResponse": {
                        "name": message.get("name"),
                        "response": (
                            message.get("content")
                            if isinstance(message.get("content"), dict)
                            else {"result": message.get("content")}
                        ),
                    }
                }
            )
            continue
        flush_responses()
        if role == "assistant":
            raw_parts = message.get("raw_parts")
            if isinstance(raw_parts, list) and raw_parts:
                parts = raw_parts
            else:
                parts = _assistant_parts(message)
            contents.append({"role": "model", "parts": parts})
            continue
        contents.append(
            {"role": "user", "parts": [{"text": str(message.get("content") or "")}]}
        )
    flush_responses()

    body: dict[str, Any] = {"contents": contents}
    if system_parts:
        body["systemInstruction"] = {"parts": [{"text": "\n".join(system_parts)}]}
    if tools:
        body["tools"] = [
            {
                "functionDeclarations": [
                    {
                        "name": tool["name"],
                        "description": tool.get("description") or "",
                        "parameters": _parameters(tool.get("parameters")),
                    }
                    for tool in tools
                ]
            }
        ]
    return url, headers, body


def _assistant_parts(message: Mapping[str, Any]) -> list[dict[str, Any]]:
    parts: list[dict[str, Any]] = []
    text = message.get("content") or ""
    if text:
        parts.append({"text": str(text)})
    for call in message.get("tool_calls") or []:
        parts.append(
            {
                "functionCall": {
                    "name": call.get("name"),
                    "args": call.get("arguments") or {},
                }
            }
        )
    if not parts:
        parts.append({"text": ""})
    return parts


def _parameters(schema: Any) -> dict[str, Any]:
    if isinstance(schema, dict) and schema.get("type") == "object":
        return schema
    return {"type": "object", "properties": {}}


def parse_gemini_response(payload: Mapping[str, Any]) -> dict[str, Any]:
    candidates = payload.get("candidates") or []
    if not candidates:
        raise DirectToolLoopError("model returned no candidates")
    content = candidates[0].get("content") or {}
    parts = content.get("parts") or []
    texts: list[str] = []
    calls: list[dict[str, Any]] = []
    for index, part in enumerate(parts):
        if not isinstance(part, dict):
            continue
        if part.get("thought") is True:
            continue
        if "text" in part and isinstance(part["text"], str):
            texts.append(part["text"])
        function_call = part.get("functionCall")
        if isinstance(function_call, dict) and function_call.get("name"):
            arguments = function_call.get("args") or {}
            if not isinstance(arguments, dict):
                arguments = {}
            name = str(function_call["name"])
            calls.append(
                {
                    "id": f"{name}-{index}",
                    "name": name,
                    "arguments": arguments,
                }
            )
    return {"text": "".join(texts), "tool_calls": calls, "raw_parts": parts}


def gemini_complete(
    messages: Sequence[Mapping[str, Any]],
    tools: Optional[Sequence[Mapping[str, Any]]],
) -> dict[str, Any]:
    url, headers, body = build_gemini_request(messages, tools, gemini_api_key())
    payload = _post_json(url, headers, body)
    return parse_gemini_response(payload)


def _post_json(url: str, headers: Mapping[str, str], body: Mapping[str, Any]) -> dict:
    assert_stable_url(url)
    request = Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers=dict(headers),
        method="POST",
    )
    try:
        with urlopen(request, timeout=90) as response:
            raw = response.read().decode("utf-8")
    except Exception as exc:
        raise DirectToolLoopError("model request failed") from exc
    try:
        loaded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise DirectToolLoopError("model returned invalid JSON") from exc
    if not isinstance(loaded, dict):
        raise DirectToolLoopError("model returned an unexpected payload")
    return loaded


def mcp_tool_declarations() -> list[dict[str, Any]]:
    """Schemas of the local MCP server, including the actuation gate as it is."""
    import asyncio

    from pib_mcp_server.server import create_server

    server = create_server()
    tools = asyncio.run(server.list_tools())
    declarations: list[dict[str, Any]] = []
    for tool in tools:
        schema = _tool_input_schema(tool)
        declarations.append(
            {
                "name": tool.name,
                "description": getattr(tool, "description", "") or "",
                "parameters": _parameters(schema),
            }
        )
    return declarations


def execute_mcp_tool(name: str, arguments: Mapping[str, Any]) -> Any:
    """Run one tool on the local MCP server. The actuation gate stays there."""
    import asyncio

    from pib_mcp_server.server import create_server

    server = create_server()
    result = asyncio.run(server.call_tool(name, dict(arguments)))
    return _unwrap_tool_result(result)


def _tool_input_schema(tool: Any) -> Any:
    schema = getattr(tool, "input_schema", None)
    if schema is None:
        schema = getattr(tool, "inputSchema", None)
    if isinstance(schema, dict):
        return schema
    if hasattr(schema, "model_dump"):
        return schema.model_dump()
    dumped = tool.model_dump() if hasattr(tool, "model_dump") else {}
    return dumped.get("inputSchema") or dumped.get("input_schema") or {}


def _unwrap_tool_result(result: Any) -> Any:
    if isinstance(result, tuple):
        structured = result[1] if len(result) > 1 else None
        if structured is not None:
            return structured
        result = result[0]
    structured_output = getattr(result, "structured_output", None)
    if structured_output is not None:
        return structured_output
    content = getattr(result, "content", None)
    if content:
        text = getattr(content[0], "text", None)
        if isinstance(text, str):
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                return {"text": text}
    return result
