"""Direct chat tool loop.

A Direct turn asks the model, runs MCP tool calls, and continues until the
model returns a final answer. The per-personality switch turns that off: the
request then carries no tools and no image. A camera frame is only available
as the result of the capture_image tool.

Pinned, checked on 2026-09-30: ``gemini-3.8-flash``, the catalogue's Gemini
chat model. It is pinned because that is the supported model this loop
calls, and the id it replaced is not in the catalogue. The endpoint is
``/v1/models/<model>:generateContent``. The beta surface is not used.
"""

from __future__ import annotations

import http.client
import json
import logging
import os
import socket
import time
from threading import Lock
from typing import Any, Callable, Iterator, Mapping, Optional, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

from pib_hermes_config.retired_direct_text import DIRECT_TEXT_MODEL_API_NAME
from voice_assistant.llm_defaults import gemini_thinking_config
from pib_hermes_config.visible_state import (
    ANSWER_GESTURE_TOOLS,
    issue_answer_gesture,
)

logger = logging.getLogger(__name__)

PINNED_PROVIDER = "gemini"
# Pinned, checked on 2026-09-30: the catalogue's Gemini chat model. The id this
# replaced is not in the catalogue, so this loop does not send it. The name
# comes from the shared Direct-transport constant, so the retirement rule and
# this loop can not disagree about which model has a transport.
PINNED_MODEL = DIRECT_TEXT_MODEL_API_NAME
PINNED_CHECKED_ON = "2026-09-30"
PINNED_ENDPOINT = (
    "https://generativelanguage.googleapis.com/v1/models/"
    f"{PINNED_MODEL}:generateContent"
)
DIRECT_STREAM_PATH = PINNED_ENDPOINT.replace(
    ":generateContent", ":streamGenerateContent?alt=sse"
)
DIRECT_REQUEST_TIMEOUT_SEC = 180
DIRECT_RETRY_DELAY_SEC = 2
# The answer ceiling. Measured: with thinking on, one request spent 982 thought tokens
# against a ceiling of 1024 and the answer came back empty. Thinking is off by default
# since PR-2039; the ceiling still has to hold an answer plus a later re-enable.
DIRECT_MAX_OUTPUT_TOKENS = 2048
# The allowlist is the pinned catalogue model. Hermes is a channel, not a row.
GEMINI_API_NAMES = frozenset({PINNED_MODEL})
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


KEY_SOURCE_STORE = "key-store"
KEY_SOURCE_ENVIRONMENT = "environment"

#: Providers Hermes can key from the store, and the env var each reads. The
#: list mirrors pib_hermes_config's provider mapping; the on-device model is
#: absent on purpose, it needs no key.
PROVIDER_KEY_ENV_VARS = {
    "gemini": ("GOOGLE_API_KEY", "GEMINI_API_KEY"),
    "openai": ("OPENAI_API_KEY",),
    "anthropic": ("ANTHROPIC_API_KEY",),
    "pib-cloud": ("PIB_CLOUD_API_KEY",),
}
# Built-in providers key per catalogue model id; custom routes (pib.Cloud) key
# by provider slug, matching cloud_token and the credential HTTP route.
_BUILTIN_KEYED_PROVIDERS = frozenset({"gemini", "openai", "anthropic"})


def missing_key_message(provider: str) -> str:
    """Names the provider. The message never carries a secret."""
    return f"No keys are available for provider {provider}."


def environment_key(provider: str) -> str:
    """Development fallback. Only this provider's own variables.

    A locked store may use these. An unlocked store does not, and another
    provider's variable is never read.
    """
    for name in PROVIDER_KEY_ENV_VARS.get(provider, ()):
        value = os.environ.get(name)
        if value:
            return value
    return ""


def gemini_api_key() -> str:
    """Environment names the voice container already receives from password.env.

    Memory consolidation still uses this. A Direct turn uses
    ``resolve_provider_key``, which prefers the key store.
    """
    return environment_key(PINNED_PROVIDER)


def _log_key_source(source: str, provider: str, channel: str = "direct") -> None:
    logger.info("%s provider key source=%s provider=%s", channel, source, provider)


def _store_credential_id(provider: str, api_name: Optional[str] = None) -> str:
    """Catalogue id passed to the credential route for this turn's provider.

    Built-in providers (Gemini, OpenAI, Anthropic) store keys per model
    ``api_name``. A custom route such as pib.Cloud is always addressed by its
    provider slug, even when a stale profile still names a pinned Gemini model.
    """
    if provider not in _BUILTIN_KEYED_PROVIDERS:
        return provider
    if isinstance(api_name, str) and api_name.strip():
        return api_name.strip()
    if provider == PINNED_PROVIDER:
        return PINNED_MODEL
    return provider


def _fetch_store_key(provider: str, api_name: Optional[str] = None) -> dict[str, Any]:
    """The unlocked secret for this provider/model, or the mode with no secret.

    The store is addressed by the model's api_name for built-in providers, or by
    the provider slug for a custom keyed route. A provider this loop does not
    key, and an unknown model, yield no secret.
    """
    if provider not in PROVIDER_KEY_ENV_VARS:
        return {"mode": "unlocked", "secret": None}
    name = _store_credential_id(provider, api_name)
    try:
        from pib_api_client.key_store_client import read_provider_key
    except Exception:
        return {"mode": "unavailable", "secret": None}
    return read_provider_key(name)


def resolve_provider_key(
    provider: str,
    fetch: Optional[Callable[[str], Mapping[str, Any]]] = None,
    *,
    api_name: Optional[str] = None,
    allow_environment: bool = True,
    log_channel: str = "direct",
) -> tuple[str, str]:
    """Key for this provider, and ``key-store`` or ``environment``.

    An unlocked store is the only source: a missing key fails, and the
    environment is not consulted. A locked or unreachable store uses this
    provider's environment variable when ``allow_environment`` is true.
    Hermes passes false, so a locked store fails instead of changing
    provider. The secret is not logged.
    """
    if not isinstance(provider, str) or provider.strip() == "":
        raise DirectToolLoopError("No keys are available for provider.")
    reader = fetch or _fetch_store_key
    try:
        state = reader(provider, api_name) if api_name else reader(provider)
    except Exception:
        state = {"mode": "unavailable", "secret": None}
    if not isinstance(state, dict):
        state = {"mode": "unavailable", "secret": None}
    mode = state.get("mode")
    if mode == "unlocked":
        secret = state.get("secret")
        if isinstance(secret, str) and secret.strip():
            _log_key_source(KEY_SOURCE_STORE, provider, log_channel)
            return secret, KEY_SOURCE_STORE
        raise DirectToolLoopError(missing_key_message(provider))
    if not allow_environment:
        raise DirectToolLoopError(missing_key_message(provider))
    env_key = environment_key(provider)
    if env_key:
        _log_key_source(KEY_SOURCE_ENVIRONMENT, provider, log_channel)
        return env_key, KEY_SOURCE_ENVIRONMENT
    raise DirectToolLoopError(missing_key_message(provider))


def resolve_hermes_provider_key(
    provider: str = PINNED_PROVIDER,
    fetch: Optional[Callable[[str], Mapping[str, Any]]] = None,
    *,
    api_name: Optional[str] = None,
) -> tuple[str, str]:
    """Key for a Hermes turn. The store is the only source.

    The Direct helper above is the read. This caller refuses the environment
    fallback, so a locked store cannot select another provider's variable.
    ``provider``/``api_name`` are the personality's model and provider.
    """
    return resolve_provider_key(
        provider,
        fetch,
        api_name=api_name,
        allow_environment=False,
        log_channel="hermes",
    )


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
    on_tool_call: Optional[Callable[[dict], None]] = None,
    on_tool_start: Optional[Callable[[dict], None]] = None,
    on_tool_image: Optional[Callable[[dict], None]] = None,
    on_tool_round: Optional[Callable[[], None]] = None,
    max_rounds: int = MAX_TOOL_ROUNDS,
    api_key: Optional[str] = None,
    report_key_source: Optional[Callable[[str], None]] = None,
    session: Optional[Any] = None,
) -> Iterator[str]:
    """Yield the model's final answer. Tool results are not spoken.

    ``history`` entries are ``(content, is_user)``. The user message is text
    only: nothing in this function attaches an image.

    A tool can return a picture (the camera tool answers ``imageBase64`` and
    ``mimeType``). It is handed to ``on_tool_image`` instead of the tool record,
    because the record of a tool call travels to the chat and carries no values:
    only a dedicated image channel can carry one without carrying arguments too.
    """
    if session is not None:
        stable_prompt = getattr(session, "stable_system_prompt", None)
        if callable(stable_prompt):
            system_prompt = stable_prompt(system_prompt)

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
    if complete is not None:
        caller = complete
    else:
        if api_key is None:
            if session is not None:
                resolve = getattr(session, "resolve_api_key", None)
                if callable(resolve):
                    api_key, source = resolve(PINNED_PROVIDER)
                else:
                    api_key, source = resolve_provider_key(PINNED_PROVIDER)
            else:
                api_key, source = resolve_provider_key(PINNED_PROVIDER)
            if report_key_source is not None:
                report_key_source(source)

        def caller(messages, tools, _key=api_key, _force=False):
            return gemini_complete_stream(
                messages,
                tools,
                api_key=_key,
                force_no_tools=_force,
            )

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

    tool_call_counter = 0

    for _round in range(max_rounds):
        result = caller(messages, tools if tool_calling else None)
        calls = result.get("tool_calls") or []
        if not tool_calling or not calls:
            yield result.get("text") or ""
            return
        if on_tool_round is not None:
            try:
                on_tool_round()
            except Exception:  # noqa: BLE001 - reporting must never fail a turn
                logger.warning("tool round reporter failed", exc_info=True)
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
            call_id = _tool_call_id(call, name, tool_call_counter)
            tool_call_counter += 1
            logger.info("direct tool call name=%s", name)
            _report_tool_start(
                on_tool_start,
                name=name,
                call_id=call_id,
                arguments=arguments,
            )
            started = time.monotonic()
            ok_call = True
            error_name = ""
            try:
                if name in ANSWER_GESTURE_TOOLS:
                    output = issue_answer_gesture(name, arguments, runner)
                else:
                    output = runner(name, arguments)
            except Exception as exc:
                ok_call = False
                error_name = type(exc).__name__
                _report_tool_call(
                    on_tool_call,
                    name=name,
                    call_id=call_id,
                    arguments=arguments,
                    started=started,
                    ok=False,
                    error=error_name,
                )
                raise
            _report_tool_call(
                on_tool_call,
                name=name,
                call_id=call_id,
                arguments=arguments,
                started=started,
                ok=ok_call,
                error=error_name,
            )
            _report_tool_image(on_tool_image, name, output)
            messages.append(
                {
                    "role": "tool",
                    "name": name,
                    "tool_call_id": call.get("id") or name,
                    "content": _tool_result_for_model(output),
                }
            )
    logger.info("direct tool rounds exhausted, forcing a final answer")
    if complete is not None:
        result = complete(messages, None)
    else:
        result = caller(messages, tools if tool_calling else None, _force=True)
    yield result.get("text") or ""


def _tool_call_id(call: Mapping[str, Any], name: str, index: int) -> str:
    """Stable id for pairing start and end trace events on one tool invocation."""
    raw = call.get("id")
    if isinstance(raw, str) and raw.strip():
        return raw.strip()
    return f"{name}-{index}"


def _report_tool_start(
    reporter: Optional[Callable[[dict], None]],
    *,
    name: str,
    call_id: str,
    arguments: Mapping[str, Any],
) -> None:
    """Tell the caller a tool is about to run, without argument values."""
    if reporter is None:
        return
    record: dict = {
        "name": name,
        "callId": call_id,
        "argNames": sorted(str(key) for key in (arguments or {})),
    }
    try:
        reporter(record)
    except Exception:  # noqa: BLE001 - reporting must never fail a turn
        logger.warning("tool start reporter failed name=%s", name, exc_info=True)


def _report_tool_call(
    reporter: Optional[Callable[[dict], None]],
    *,
    name: str,
    call_id: str,
    arguments: Mapping[str, Any],
    started: float,
    ok: bool,
    error: str,
) -> None:
    """Tell the caller about one executed tool, without its argument values.

    Only the argument *names* travel: a tool argument can carry a location, a
    message or a key, and none of that belongs in a chat message, a trace event or
    a log line. A reporter that raises must not break the turn.
    """
    if reporter is None:
        return
    record: dict = {
        "name": name,
        "callId": call_id,
        "ok": ok,
        "durationMs": round((time.monotonic() - started) * 1000.0, 1),
        "argNames": sorted(str(key) for key in (arguments or {})),
    }
    if error:
        record["error"] = error
    try:
        reporter(record)
    except Exception:  # noqa: BLE001 - reporting must never fail a turn
        logger.warning("tool call reporter failed name=%s", name, exc_info=True)


def _report_tool_image(
    reporter: Optional[Callable[[dict], None]], name: str, output: Any
) -> None:
    """Hand a captured picture to the image channel, and only the picture.

    A tool result is a mapping; the camera tool puts ``imageBase64`` and
    ``mimeType`` in it. Anything else - a motor position, a program list, an error
    - is not an image and never reaches this channel, so the chat cannot
    accidentally display a tool's other output.
    """
    if reporter is None or not isinstance(output, Mapping):
        return
    image = output.get("imageBase64")
    if not isinstance(image, str) or not image.strip():
        return
    mime_type = output.get("mimeType")
    reporter(
        {
            "tool": name,
            "mimeType": mime_type if isinstance(mime_type, str) else "image/jpeg",
            "imageBase64": image,
        }
    )


def _json_object(value: Any) -> dict[str, Any]:
    """Gemini function responses have to be objects."""
    if isinstance(value, dict):
        return value
    return {"result": value}


def _tool_result_for_model(output: Any) -> dict[str, Any]:
    """What the model is told after a tool ran, without a picture's bytes.

    A camera tool answers with a base64 frame - measured around 100 000 characters
    for one 640x480 capture. Echoing that back as the function response made the
    request after a capture fail (``model request failed``), and the model gains
    nothing from it: it asked for the frame and has already described it. The
    picture reaches the chat through ``on_tool_image``; what travels back here is a
    note that it exists.
    """
    result = _json_object(output)
    image = result.get("imageBase64")
    if not isinstance(image, str) or not image.strip():
        return result
    compact = {key: value for key, value in result.items() if key != "imageBase64"}
    compact["imageCaptured"] = True
    compact["imageEncodedLength"] = len(image)
    return compact


def build_gemini_request(
    messages: Sequence[Mapping[str, Any]],
    tools: Optional[Sequence[Mapping[str, Any]]],
    api_key: str,
    *,
    stream: bool = False,
    force_no_tools: bool = False,
) -> tuple[str, dict[str, str], dict[str, Any]]:
    """Stable generateContent request. The key travels in a header, not the URL."""
    url = assert_stable_url(DIRECT_STREAM_PATH if stream else PINNED_ENDPOINT)
    if not api_key:
        raise DirectToolLoopError(missing_key_message(PINNED_PROVIDER))
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

    body: dict[str, Any] = {
        "contents": contents,
        "generationConfig": {
            "maxOutputTokens": DIRECT_MAX_OUTPUT_TOKENS,
            "thinkingConfig": gemini_thinking_config(),
        },
    }
    if system_parts:
        body["systemInstruction"] = {"parts": [{"text": "\n".join(system_parts)}]}
    if force_no_tools:
        body["toolConfig"] = {"functionCallingConfig": {"mode": "NONE"}}
    elif tools:
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


def _part_is_answer_text(part: Mapping[str, Any]) -> bool:
    if part.get("thought") is True:
        return False
    if "thoughtSignature" in part:
        return False
    return isinstance(part.get("text"), str)


def _result_from_parts(parts: Sequence[Any]) -> dict[str, Any]:
    texts: list[str] = []
    calls: list[dict[str, Any]] = []
    raw_parts: list[Any] = []
    for part in parts:
        if not isinstance(part, dict):
            continue
        raw_parts.append(part)
        if _part_is_answer_text(part):
            texts.append(part["text"])
        function_call = part.get("functionCall")
        if isinstance(function_call, dict) and function_call.get("name"):
            arguments = function_call.get("args") or {}
            if not isinstance(arguments, dict):
                arguments = {}
            name = str(function_call["name"])
            calls.append(
                {
                    # Gemini gives a function call no id. Inventing one from the
                    # part index made every repeated call of the same tool in one
                    # turn share an id (`move_motor-0` six times), and the chat
                    # window pairs start and finish by id, so only the first of
                    # them was ever closed (PR-2042). Leave it out: run_direct_turn
                    # assigns the per-turn counter in _tool_call_id.
                    "name": name,
                    "arguments": arguments,
                }
            )
    return {"text": "".join(texts), "tool_calls": calls, "raw_parts": raw_parts}


def parse_gemini_response(payload: Mapping[str, Any]) -> dict[str, Any]:
    candidates = payload.get("candidates") or []
    if not candidates:
        raise DirectToolLoopError("model returned no candidates")
    content = candidates[0].get("content") or {}
    parts = content.get("parts") or []
    return _result_from_parts(parts)


def _accumulate_sse_chunk(
    accumulated: list[dict[str, Any]],
    payload: Mapping[str, Any],
    on_delta: Optional[Callable[[str], None]] = None,
) -> None:
    candidates = payload.get("candidates") or []
    if not candidates:
        return
    content = candidates[0].get("content") or {}
    for part in content.get("parts") or []:
        if not isinstance(part, dict):
            continue
        accumulated.append(part)
        if on_delta is not None and _part_is_answer_text(part):
            on_delta(part["text"])


def parse_gemini_sse_stream(
    raw_stream: str,
    on_delta: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    """Reassemble a Gemini ``alt=sse`` body into the same shape as ``parse_gemini_response``."""
    parts: list[dict[str, Any]] = []
    for line in raw_stream.splitlines():
        stripped = line.strip()
        if not stripped.startswith("data:"):
            continue
        payload_text = stripped[len("data:") :].strip()
        if not payload_text or payload_text == "[DONE]":
            continue
        try:
            chunk = json.loads(payload_text)
        except json.JSONDecodeError:
            continue
        if isinstance(chunk, dict):
            _accumulate_sse_chunk(parts, chunk, on_delta=on_delta)
    return _result_from_parts(parts)


def _read_sse_response(
    response: Any,
    on_delta: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    parts: list[dict[str, Any]] = []
    for raw_line in response:
        if isinstance(raw_line, bytes):
            line = raw_line.decode("utf-8", errors="replace")
        else:
            line = str(raw_line)
        stripped = line.strip()
        if not stripped.startswith("data:"):
            continue
        payload_text = stripped[len("data:") :].strip()
        if not payload_text or payload_text == "[DONE]":
            continue
        try:
            chunk = json.loads(payload_text)
        except json.JSONDecodeError:
            continue
        if isinstance(chunk, dict):
            _accumulate_sse_chunk(parts, chunk, on_delta=on_delta)
    if not parts:
        raise DirectToolLoopError("model returned no candidates")
    return _result_from_parts(parts)


def gemini_complete(
    messages: Sequence[Mapping[str, Any]],
    tools: Optional[Sequence[Mapping[str, Any]]],
    api_key: Optional[str] = None,
) -> dict[str, Any]:
    key = gemini_api_key() if api_key is None else api_key
    url, headers, body = build_gemini_request(messages, tools, key)
    payload = _post_json(url, headers, body)
    return parse_gemini_response(payload)


def gemini_complete_stream(
    messages: Sequence[Mapping[str, Any]],
    tools: Optional[Sequence[Mapping[str, Any]]],
    api_key: Optional[str] = None,
    on_delta: Optional[Callable[[str], None]] = None,
    *,
    force_no_tools: bool = False,
) -> dict[str, Any]:
    key = gemini_api_key() if api_key is None else api_key
    url, headers, body = build_gemini_request(
        messages,
        tools,
        key,
        stream=True,
        force_no_tools=force_no_tools,
    )
    return _post_sse(url, headers, body, on_delta=on_delta)


def _transport_retry_reason(exc: BaseException) -> Optional[str]:
    if isinstance(exc, (TimeoutError, socket.timeout)):
        return f"{type(exc).__name__}: {str(exc)[:200]}"
    if isinstance(exc, URLError) and not isinstance(exc.reason, HTTPError):
        reason = exc.reason
        return f"{type(exc).__name__}: {str(reason)[:200]}"
    return None


def _with_transport_retry(operation: Callable[[], Any]) -> Any:
    last_exc: Optional[BaseException] = None
    reason = ""
    for attempt in range(2):
        try:
            return operation()
        except HTTPError:
            raise
        except Exception as exc:
            retry_reason = _transport_retry_reason(exc)
            if retry_reason is None:
                raise
            last_exc = exc
            reason = retry_reason
            if attempt == 0:
                logger.info("model request retried after %s", reason)
                time.sleep(DIRECT_RETRY_DELAY_SEC)
                continue
            break
    assert last_exc is not None
    logger.error(
        "model request failed: %s: %s", type(last_exc).__name__, str(last_exc)[:300]
    )
    raise DirectToolLoopError(
        f"model request failed: {type(last_exc).__name__}: {str(last_exc)[:200]}"
    ) from last_exc


def _raise_http_error(exc: HTTPError) -> None:
    body = ""
    try:
        body = exc.read().decode("utf-8", "replace")[:300]
    except Exception:  # pragma: no cover - a body that cannot be read
        body = "<unreadable>"
    logger.error("model request failed: HTTP %s %s body=%s", exc.code, exc.reason, body)
    raise DirectToolLoopError(
        f"model request failed: HTTP {exc.code} {exc.reason}"
    ) from exc


_gemini_transport_lock = Lock()
_gemini_keepalive_transport: Optional["_GeminiKeepAliveTransport"] = None
_gemini_connections_opened = 0


def reset_gemini_http_transport() -> None:
    """Drop the shared Gemini HTTP connection (tests and key rotation)."""
    global _gemini_keepalive_transport, _gemini_connections_opened
    with _gemini_transport_lock:
        if _gemini_keepalive_transport is not None:
            _gemini_keepalive_transport.close()
            _gemini_keepalive_transport = None
        _gemini_connections_opened = 0


def gemini_http_connections_opened() -> int:
    """How many TLS connections the keep-alive transport has opened."""
    with _gemini_transport_lock:
        return _gemini_connections_opened


def _keepalive_enabled() -> bool:
    return os.environ.get("DIRECT_HTTP_KEEPALIVE", "1") != "0"


class _GeminiKeepAliveTransport:
    """One HTTPS connection to generativelanguage.googleapis.com per process."""

    def __init__(self) -> None:
        self._host: Optional[str] = None
        self._conn: Optional[http.client.HTTPSConnection] = None

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001 - best-effort close
                pass
        self._conn = None
        self._host = None

    def _ensure_connection(self, host: str) -> http.client.HTTPSConnection:
        global _gemini_connections_opened
        if self._conn is None or self._host != host:
            self.close()
            self._host = host
            self._conn = http.client.HTTPSConnection(
                host, timeout=DIRECT_REQUEST_TIMEOUT_SEC
            )
            _gemini_connections_opened += 1
        return self._conn

    def post_json(self, url: str, headers: Mapping[str, str], body: Mapping[str, Any]) -> dict:
        parsed = urlparse(url)
        payload = json.dumps(body).encode("utf-8")
        request_headers = dict(headers)
        request_headers.setdefault("Connection", "keep-alive")
        conn = self._ensure_connection(parsed.netloc)
        try:
            conn.request(
                "POST",
                parsed.path + (f"?{parsed.query}" if parsed.query else ""),
                body=payload,
                headers=request_headers,
            )
            response = conn.getresponse()
            raw = response.read().decode("utf-8")
            if response.status >= 400:
                raise DirectToolLoopError(
                    f"model request failed: HTTP {response.status} {response.reason}"
                )
        except DirectToolLoopError:
            raise
        except Exception as exc:
            self.close()
            retry_reason = _transport_retry_reason(exc)
            if retry_reason is not None:
                raise
            logger.error(
                "model request failed: %s: %s", type(exc).__name__, str(exc)[:300]
            )
            raise DirectToolLoopError(
                f"model request failed: {type(exc).__name__}: {str(exc)[:200]}"
            ) from exc
        try:
            loaded = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise DirectToolLoopError("model returned invalid JSON") from exc
        if not isinstance(loaded, dict):
            raise DirectToolLoopError("model returned an unexpected payload")
        return loaded

    def post_sse(
        self,
        url: str,
        headers: Mapping[str, str],
        body: Mapping[str, Any],
        on_delta: Optional[Callable[[str], None]] = None,
    ) -> dict[str, Any]:
        parsed = urlparse(url)
        payload = json.dumps(body).encode("utf-8")
        request_headers = dict(headers)
        request_headers.setdefault("Connection", "keep-alive")
        conn = self._ensure_connection(parsed.netloc)
        try:
            conn.request(
                "POST",
                parsed.path + (f"?{parsed.query}" if parsed.query else ""),
                body=payload,
                headers=request_headers,
            )
            response = conn.getresponse()
            if response.status >= 400:
                body_preview = response.read(300).decode("utf-8", errors="replace")
                logger.error(
                    "model request failed: HTTP %s %s body=%s",
                    response.status,
                    response.reason,
                    body_preview,
                )
                raise DirectToolLoopError(
                    f"model request failed: HTTP {response.status} {response.reason}"
                )
            return _read_sse_response(response, on_delta=on_delta)
        except HTTPError as exc:
            _raise_http_error(exc)
        except DirectToolLoopError:
            raise
        except Exception as exc:
            self.close()
            retry_reason = _transport_retry_reason(exc)
            if retry_reason is not None:
                raise
            logger.error(
                "model request failed: %s: %s", type(exc).__name__, str(exc)[:300]
            )
            raise DirectToolLoopError(
                f"model request failed: {type(exc).__name__}: {str(exc)[:200]}"
            ) from exc


def _get_gemini_keepalive_transport() -> _GeminiKeepAliveTransport:
    global _gemini_keepalive_transport
    with _gemini_transport_lock:
        if _gemini_keepalive_transport is None:
            _gemini_keepalive_transport = _GeminiKeepAliveTransport()
        return _gemini_keepalive_transport


def _post_json_urlopen(
    url: str, headers: Mapping[str, str], body: Mapping[str, Any]
) -> dict:
    request = Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers=dict(headers),
        method="POST",
    )
    try:
        with urlopen(request, timeout=DIRECT_REQUEST_TIMEOUT_SEC) as response:
            raw = response.read().decode("utf-8")
    except HTTPError as exc:
        _raise_http_error(exc)
    except Exception as exc:
        retry_reason = _transport_retry_reason(exc)
        if retry_reason is not None:
            raise
        logger.error(
            "model request failed: %s: %s", type(exc).__name__, str(exc)[:300]
        )
        raise DirectToolLoopError(
            f"model request failed: {type(exc).__name__}: {str(exc)[:200]}"
        ) from exc
    try:
        loaded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise DirectToolLoopError("model returned invalid JSON") from exc
    if not isinstance(loaded, dict):
        raise DirectToolLoopError("model returned an unexpected payload")
    return loaded


def _post_sse_urlopen(
    url: str,
    headers: Mapping[str, str],
    body: Mapping[str, Any],
    on_delta: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    request = Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers=dict(headers),
        method="POST",
    )
    try:
        with urlopen(request, timeout=DIRECT_REQUEST_TIMEOUT_SEC) as response:
            return _read_sse_response(response, on_delta=on_delta)
    except HTTPError as exc:
        _raise_http_error(exc)
    except Exception as exc:
        retry_reason = _transport_retry_reason(exc)
        if retry_reason is not None:
            raise
        logger.error(
            "model request failed: %s: %s", type(exc).__name__, str(exc)[:300]
        )
        raise DirectToolLoopError(
            f"model request failed: {type(exc).__name__}: {str(exc)[:200]}"
        ) from exc


def _post_json(url: str, headers: Mapping[str, str], body: Mapping[str, Any]) -> dict:
    assert_stable_url(url)

    def operation() -> dict:
        if _keepalive_enabled():
            return _get_gemini_keepalive_transport().post_json(url, headers, body)
        return _post_json_urlopen(url, headers, body)

    return _with_transport_retry(operation)


def _post_sse(
    url: str,
    headers: Mapping[str, str],
    body: Mapping[str, Any],
    on_delta: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    assert_stable_url(url)

    def operation() -> dict[str, Any]:
        if _keepalive_enabled():
            return _get_gemini_keepalive_transport().post_sse(
                url, headers, body, on_delta=on_delta
            )
        return _post_sse_urlopen(url, headers, body, on_delta=on_delta)

    return _with_transport_retry(operation)


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
