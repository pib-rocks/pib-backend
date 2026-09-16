"""Long-lived Hermes Agent daemon for low-latency voice assistant turns.

Exposes a localhost HTTP API so ``hermes_agent_client.run_turn`` can dispatch
to a warm process instead of cold-starting the Hermes CLI on every chat turn.

Endpoints:
  GET  /health  → 200 {"status": "ok"}
  POST /turn    → JSON {text, chat_id, personality_id?, toolsets?, max_turns?}
                  → {"reply": "..."}
                  With stream=true, returns newline-delimited delta/final JSON.
"""

from __future__ import annotations

import contextlib
import glob
import importlib
import io
import json
import logging
import os
import re
import socket
import sys
import threading
import time
from collections import OrderedDict

# Force IPv4 preference in socket.getaddrinfo to prevent 10s IPv6 timeouts on Pi networks
_orig_getaddrinfo = socket.getaddrinfo


def _ipv4_preferred_getaddrinfo(*args, **kwargs):
    res = _orig_getaddrinfo(*args, **kwargs)
    ipv4 = [r for r in res if r[0] == socket.AF_INET]
    return ipv4 if ipv4 else res


socket.getaddrinfo = _ipv4_preferred_getaddrinfo
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional
from urllib.error import URLError
from urllib.request import Request, urlopen

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8088
DEFAULT_BASE_URL = f"http://{DEFAULT_HOST}:{DEFAULT_PORT}"

# How long ensure_daemon_running waits for /health after spawning.
_STARTUP_WAIT_SECONDS = 5.0
_STARTUP_POLL_SECONDS = 0.05

# Process-local server handle started by start_daemon / ensure_daemon_running.
_server: Optional[ThreadingHTTPServer] = None
_server_thread: Optional[threading.Thread] = None
_server_lock = threading.Lock()

TurnRunner = Callable[..., str]

# Model used for the in-process ``run_agent.main`` entry point.
IN_PROCESS_MODEL = "gemini-3.5-flash"
DEFAULT_AGENT_CACHE_SIZE = 32

# Hermes install layout used to import the agent in-process.
DEFAULT_HERMES_HOME = "/home/pib/.hermes"
HERMES_AGENT_DIRNAME = "hermes-agent"
_VENV_SITE_PACKAGES_GLOB = os.path.join("venv", "lib", "python3.*", "site-packages")
_PYTHON_VERSION = re.compile(r"python(\d+)\.(\d+)")

# ``run_agent.main`` has no return value: it prints the answer to stdout below a
# "FINAL RESPONSE:" banner, followed by decorative dashes and closing status
# lines. The daemon captures that stdout and extracts the answer from it.
_FINAL_RESPONSE_MARKER = "FINAL RESPONSE:"
_DECORATION_LINE = re.compile(r"^[-=\s]*$")
_TRAILING_MARKERS = (
    "Agent execution completed",
    "Sample trajectory saved to",
    "Failed to save sample",
)

# redirect_stdout swaps a process-global, so parallel /turn requests would read
# each other's output. Serialize the captured runs.
_stdout_capture_lock = threading.Lock()


class _CachedAgent:
    def __init__(self, agent, settings: tuple[Optional[str], int]):
        self.agent = agent
        self.settings = settings
        self.lock = threading.Lock()


_agent_cache: OrderedDict[str, _CachedAgent] = OrderedDict()
_agent_cache_lock = threading.Lock()
_mcp_discovery_attempted = False
_mcp_discovery_lock = threading.Lock()


def _toolset_list(toolsets: Optional[str]) -> Optional[list[str]]:
    if not toolsets:
        return None
    values = [value.strip() for value in toolsets.split(",") if value.strip()]
    return values or None


def _close_agent(agent) -> None:
    close = getattr(agent, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            logging.debug("could not close evicted Hermes agent", exc_info=True)


def clear_agent_cache() -> None:
    """Close and remove all process-local chat agents (primarily for shutdown/tests)."""
    with _agent_cache_lock:
        entries = list(_agent_cache.values())
        _agent_cache.clear()
    for entry in entries:
        _close_agent(entry.agent)


def _discover_mcp_tools() -> None:
    """Run Hermes' bounded, non-interactive MCP startup path."""
    from hermes_cli.mcp_startup import (
        ensure_mcp_discovery_before_agent_build,
        mcp_discovery_in_flight,
    )

    ensure_mcp_discovery_before_agent_build(
        logger=logging.getLogger(__name__),
        thread_name="pib-hermes-daemon-mcp",
    )
    if mcp_discovery_in_flight():
        logging.warning(
            "Hermes MCP discovery timed out before agent construction; "
            "continuing without waiting"
        )


def _ensure_mcp_tools_discovered() -> None:
    """Attempt MCP discovery once in this daemon process, without failing turns."""
    global _mcp_discovery_attempted

    with _mcp_discovery_lock:
        if _mcp_discovery_attempted:
            return
        _mcp_discovery_attempted = True
        try:
            _discover_mcp_tools()
        except Exception as exc:
            logging.warning(
                "Hermes MCP discovery failed; continuing without MCP tools: %s",
                exc,
                exc_info=True,
            )


def _registered_mcp_tool_count() -> int:
    """Return the current process-wide MCP tool count, or zero if unavailable."""
    try:
        from tools.registry import registry

        return sum(
            1 for entry in registry.get_all_entries() if entry.name.startswith("mcp__")
        )
    except Exception:
        logging.debug("could not inspect registered Hermes MCP tools", exc_info=True)
        return 0


def _agent_for_chat(chat_id: str, toolsets: Optional[str], max_turns: int, agent_cls):
    """Return the sole cached agent for a chat, evicting least-recently-used chats."""
    settings = (toolsets, max_turns)
    with _agent_cache_lock:
        cached = _agent_cache.get(chat_id)
        if cached is not None and cached.settings == settings:
            _agent_cache.move_to_end(chat_id)
            return cached
        if cached is not None:
            _agent_cache.pop(chat_id)
            _close_agent(cached.agent)

        construction_started = time.monotonic()
        agent = agent_cls(
            model=IN_PROCESS_MODEL,
            session_id=f"pib_chat_{chat_id}",
            enabled_toolsets=None,
            disabled_toolsets=_toolset_list(toolsets),
            max_iterations=max_turns,
            platform="cli",
            skip_memory=True,
        )
        logging.info(
            "[PERF_TRACE] HERMES_AGENT_CONSTRUCTED chat=%s "
            "elapsed_ms=%.2f mcp_tools=%d",
            chat_id,
            (time.monotonic() - construction_started) * 1000.0,
            _registered_mcp_tool_count(),
        )
        cached = _CachedAgent(agent, settings)
        _agent_cache[chat_id] = cached

        while len(_agent_cache) > DEFAULT_AGENT_CACHE_SIZE:
            _evicted_chat, evicted = _agent_cache.popitem(last=False)
            _close_agent(evicted.agent)
        return cached


def extract_final_response(stdout_text: str) -> str:
    """Pull the answer text out of ``run_agent.main`` stdout.

    Returns an empty string when the banner is absent or carries no text.
    """
    if not stdout_text:
        return ""

    marker_at = stdout_text.rfind(_FINAL_RESPONSE_MARKER)
    if marker_at < 0:
        return ""

    lines = stdout_text[marker_at + len(_FINAL_RESPONSE_MARKER) :].splitlines()
    while lines and _DECORATION_LINE.match(lines[0]):
        lines.pop(0)

    body: list[str] = []
    for line in lines:
        if any(marker in line for marker in _TRAILING_MARKERS):
            break
        body.append(line)

    while body and _DECORATION_LINE.match(body[-1]):
        body.pop()

    return "\n".join(body).strip()


def _coerce_reply(reply: object) -> str:
    """Normalise the various shapes ``run_agent`` implementations return."""
    if reply is None:
        return ""
    if hasattr(reply, "content"):
        reply = reply.content
    elif isinstance(reply, dict) and "response" in reply:
        reply = reply["response"]
    if reply is None:
        return ""
    return (reply if isinstance(reply, str) else str(reply)).strip()


def hermes_agent_dir() -> str:
    """Directory of the bundled hermes-agent sources inside HERMES_HOME."""
    home = os.environ.get("HERMES_HOME") or DEFAULT_HERMES_HOME
    return os.path.join(home, HERMES_AGENT_DIRNAME)


def _running_python_version() -> tuple[int, int]:
    return sys.version_info.major, sys.version_info.minor


def _python_version_from_site_packages(path: str) -> Optional[tuple[int, int]]:
    found = _PYTHON_VERSION.search(path)
    if found is None:
        return None
    return int(found.group(1)), int(found.group(2))


def venv_site_packages(agent_dir: Optional[str] = None) -> list[str]:
    """Hermes venv site-packages whose pythonX.Y matches this interpreter.

    Native wheels in a 3.13 venv cannot load on 3.12 (and vice versa). Search
    stays version-agnostic among matching trees; several hits are returned in
    a deterministic path order. Empty when nothing matches — the in-process
    turn then uses the interpreter's own packages.
    """
    base = agent_dir or hermes_agent_dir()
    running = _running_python_version()
    try:
        matches = [
            path
            for path in glob.glob(os.path.join(base, _VENV_SITE_PACKAGES_GLOB))
            if os.path.isdir(path)
        ]
    except OSError as exc:
        logging.warning("could not scan %s for a hermes venv: %s", base, exc)
        return []

    compatible = [
        path for path in matches if _python_version_from_site_packages(path) == running
    ]
    compatible.sort()
    if compatible:
        logging.info(
            "prepending hermes venv site-packages matching Python %s.%s: %s",
            running[0],
            running[1],
            compatible,
        )
    else:
        logging.info(
            "no hermes venv site-packages matching Python %s.%s; inserting none",
            running[0],
            running[1],
        )
    return compatible


def daemon_host() -> str:
    return os.environ.get("PIB_HERMES_DAEMON_HOST") or DEFAULT_HOST


def daemon_port() -> int:
    raw = os.environ.get("PIB_HERMES_DAEMON_PORT")
    if raw:
        return int(raw)
    return DEFAULT_PORT


def daemon_base_url() -> str:
    """Base URL of the daemon (no trailing slash). Overridable via env."""
    return (
        os.environ.get("PIB_HERMES_DAEMON_URL")
        or f"http://{daemon_host()}:{daemon_port()}"
    ).rstrip("/")


def daemon_turn_url() -> str:
    return daemon_base_url() + "/turn"


def daemon_health_url() -> str:
    return daemon_base_url() + "/health"


def run_turn_in_process(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    max_turns: Optional[int] = None,
    timeout: Optional[int] = None,
    stream_callback: Optional[Callable[[str], None]] = None,
) -> str:
    """Execute one turn via a cached ``AIAgent`` dedicated to this chat.

    Hermes resolves HERMES_HOME at import time, so the cached in-process API
    cannot safely switch to a personality-specific profile. Profiles are still
    provisioned for the CLI fallback, but in-process turns use the daemon's
    startup profile.
    """
    from public_api_client.hermes_agent_client import (
        DEFAULT_DISABLED_TOOLSETS,
        DEFAULT_MAX_TURNS,
        FALLBACK_REPLY,
        run_turn_subprocess,
    )

    agent_dir = hermes_agent_dir()
    for path_entry in (agent_dir, *venv_site_packages(agent_dir)):
        if path_entry not in sys.path and os.path.exists(path_entry):
            sys.path.insert(0, path_entry)

    agent_module = None
    for module_name in ("run_agent", "hermes.run_agent"):
        try:
            agent_module = importlib.import_module(module_name)
            break
        except ImportError:
            continue
    agent_cls = getattr(agent_module, "AIAgent", None)
    run_agent_main = getattr(agent_module, "main", None)

    effective_toolsets = DEFAULT_DISABLED_TOOLSETS if toolsets is None else toolsets
    effective_max_turns = DEFAULT_MAX_TURNS if max_turns is None else max_turns

    def _subprocess_reply() -> str:
        kwargs = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": effective_toolsets,
        }
        if timeout is not None:
            kwargs["timeout"] = timeout
        return run_turn_subprocess(**kwargs)

    if agent_cls is None and run_agent_main is None:
        logging.info(
            "Hermes Python API unavailable; falling back to CLI subprocess (chat=%s)",
            chat_id,
        )
        return _subprocess_reply()

    try:
        if agent_cls is not None:
            _ensure_mcp_tools_discovered()
            cached = _agent_for_chat(
                chat_id, effective_toolsets, effective_max_turns, agent_cls
            )
            produced: list[str] = []

            def capture_delta(delta: str) -> None:
                if isinstance(delta, str) and delta:
                    produced.append(delta)
                    if stream_callback is not None:
                        stream_callback(delta)

            with cached.lock:
                reply = _coerce_reply(
                    cached.agent.chat(text, stream_callback=capture_delta)
                )
            if not reply:
                reply = "".join(produced).strip()
        else:
            captured = io.StringIO()
            with _stdout_capture_lock, contextlib.redirect_stdout(captured):
                returned = run_agent_main(
                    query=text,
                    model=IN_PROCESS_MODEL,
                    disabled_toolsets=effective_toolsets,
                    max_turns=effective_max_turns,
                )
            reply = extract_final_response(captured.getvalue())
            if not reply:
                reply = _coerce_reply(returned)
            if not reply:
                logging.warning(
                    "no FINAL RESPONSE in run_agent stdout (chat=%s, %d chars captured)",
                    chat_id,
                    len(captured.getvalue()),
                )
    except Exception as exc:
        logging.exception(
            "in-process hermes turn failed (chat=%s): %s",
            chat_id,
            exc,
        )
        return FALLBACK_REPLY

    if reply:
        return reply

    try:
        return _subprocess_reply() or FALLBACK_REPLY
    except Exception as exc:
        logging.exception(
            "subprocess fallback after empty in-process reply failed (chat=%s): %s",
            chat_id,
            exc,
        )
        return FALLBACK_REPLY


def _default_turn_runner(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    max_turns: Optional[int] = None,
    timeout: Optional[int] = None,
    stream_callback: Optional[Callable[[str], None]] = None,
) -> str:
    """Execute one turn in-process via Hermes Agent (subprocess fallback)."""
    return run_turn_in_process(
        text=text,
        chat_id=chat_id,
        personality_id=personality_id,
        toolsets=toolsets,
        max_turns=max_turns,
        timeout=timeout,
        stream_callback=stream_callback,
    )


class HermesDaemonHandler(BaseHTTPRequestHandler):
    """Minimal request handler for /health and /turn."""

    # Injected on the server instance before serve_forever.
    turn_runner: TurnRunner = staticmethod(_default_turn_runner)  # type: ignore[assignment]

    def log_message(self, fmt: str, *args) -> None:
        logging.debug("hermes-daemon: " + fmt, *args)

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _start_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
        self.end_headers()

    def _send_stream_chunk(self, payload: dict) -> None:
        self.wfile.write((json.dumps(payload) + "\n").encode("utf-8"))
        self.wfile.flush()

    def do_GET(self) -> None:  # noqa: N802 — http.server API
        if self.path.rstrip("/") == "/health":
            self._send_json(200, {"status": "ok"})
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802 — http.server API
        if self.path.rstrip("/") != "/turn":
            self._send_json(404, {"error": "not found"})
            return

        t0 = time.monotonic()
        logging.info("[PERF_TRACE] DAEMON_RECV elapsed_ms=0.00")

        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8") or "{}")
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"error": "invalid json"})
            return

        if not isinstance(data, dict):
            self._send_json(400, {"error": "body must be a json object"})
            return

        text = data.get("text")
        chat_id = data.get("chat_id")
        if not isinstance(text, str) or not isinstance(chat_id, str):
            self._send_json(400, {"error": "text and chat_id are required strings"})
            return

        personality_id = data.get("personality_id")
        toolsets = data.get("toolsets")
        max_turns = data.get("max_turns")
        timeout = data.get("timeout")
        stream = data.get("stream", False)
        if personality_id is not None and not isinstance(personality_id, str):
            self._send_json(400, {"error": "personality_id must be a string"})
            return
        if toolsets is not None and not isinstance(toolsets, str):
            self._send_json(400, {"error": "toolsets must be a string"})
            return
        if max_turns is not None and (
            not isinstance(max_turns, int)
            or isinstance(max_turns, bool)
            or max_turns < 1
        ):
            self._send_json(400, {"error": "max_turns must be a positive integer"})
            return
        if timeout is not None and not isinstance(timeout, (int, float)):
            self._send_json(400, {"error": "timeout must be a number"})
            return
        if not isinstance(stream, bool):
            self._send_json(400, {"error": "stream must be a boolean"})
            return

        runner = getattr(self.server, "turn_runner", None) or _default_turn_runner
        turn_start = time.monotonic()
        logging.info(
            "[PERF_TRACE] DAEMON_TURN_START chat=%s elapsed_ms=%.2f",
            chat_id,
            (turn_start - t0) * 1000.0,
        )
        first_delta = False

        def emit_delta(delta: str) -> None:
            nonlocal first_delta
            if not isinstance(delta, str) or not delta:
                return
            if not first_delta:
                first_delta = True
                logging.info(
                    "[PERF_TRACE] DAEMON_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
                    chat_id,
                    (time.monotonic() - t0) * 1000.0,
                )
            self._send_stream_chunk({"delta": delta})

        if stream:
            self._start_stream()

        runner_kwargs = {
            "text": text,
            "chat_id": chat_id,
            "personality_id": personality_id,
            "toolsets": toolsets,
            "timeout": int(timeout) if timeout is not None else None,
        }
        if max_turns is not None:
            runner_kwargs["max_turns"] = max_turns
        if stream:
            runner_kwargs["stream_callback"] = emit_delta

        try:
            reply = runner(**runner_kwargs)
        except Exception as exc:
            logging.exception("hermes-daemon /turn failed: %s", exc)
            if stream:
                self._send_stream_chunk({"error": str(exc)})
            else:
                self._send_json(500, {"error": str(exc)})
            return

        if not first_delta:
            logging.info(
                "[PERF_TRACE] DAEMON_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
                chat_id,
                (time.monotonic() - t0) * 1000.0,
            )
        logging.info(
            "[PERF_TRACE] DAEMON_DONE chat=%s elapsed_ms=%.2f",
            chat_id,
            (time.monotonic() - t0) * 1000.0,
        )
        payload = {"reply": reply if isinstance(reply, str) else str(reply)}
        if stream:
            self._send_stream_chunk(payload)
        else:
            self._send_json(200, payload)


def create_server(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> ThreadingHTTPServer:
    """Build a ThreadingHTTPServer bound to host:port."""
    server = ThreadingHTTPServer(
        (host or daemon_host(), port if port is not None else daemon_port()),
        HermesDaemonHandler,
    )
    server.turn_runner = turn_runner or _default_turn_runner  # type: ignore[attr-defined]
    return server


def serve_forever(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> None:
    """Block serving HTTP until interrupted."""
    server = create_server(host=host, port=port, turn_runner=turn_runner)
    logging.info(
        "hermes daemon listening on http://%s:%s",
        server.server_address[0],
        server.server_address[1],
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        clear_agent_cache()


def is_daemon_reachable(timeout: float = 0.5) -> bool:
    """True when GET /health returns HTTP 200."""
    try:
        req = Request(daemon_health_url(), method="GET")
        with urlopen(req, timeout=timeout) as resp:
            return getattr(resp, "status", 200) == 200
    except (URLError, OSError, TimeoutError, ValueError):
        return False


def start_daemon(
    host: Optional[str] = None,
    port: Optional[int] = None,
    turn_runner: Optional[TurnRunner] = None,
) -> ThreadingHTTPServer:
    """Start the daemon HTTP server in a background daemon thread.

    Idempotent within this process: a second call returns the existing server
    when it is still running. Raises OSError if the port is already taken by
    another process.
    """
    global _server, _server_thread

    with _server_lock:
        if (
            _server is not None
            and _server_thread is not None
            and _server_thread.is_alive()
        ):
            return _server

        server = create_server(host=host, port=port, turn_runner=turn_runner)
        thread = threading.Thread(
            target=server.serve_forever,
            name="hermes-daemon",
            daemon=True,
        )
        thread.start()
        _server = server
        _server_thread = thread
        logging.info(
            "hermes daemon started on http://%s:%s",
            server.server_address[0],
            server.server_address[1],
        )
        return server


def stop_daemon() -> None:
    """Shut down the in-process daemon started by start_daemon (best-effort)."""
    global _server, _server_thread

    with _server_lock:
        server = _server
        thread = _server_thread
        _server = None
        _server_thread = None

    if server is None:
        return
    try:
        server.shutdown()
    except Exception as exc:
        logging.debug("hermes daemon shutdown: %s", exc)
    try:
        server.server_close()
    except Exception:
        pass
    if thread is not None and thread.is_alive():
        thread.join(timeout=2.0)
    clear_agent_cache()


def ensure_daemon_running(
    host: Optional[str] = None,
    port: Optional[int] = None,
    wait_seconds: float = _STARTUP_WAIT_SECONDS,
) -> bool:
    """Make sure a daemon is reachable; start one in-process if needed.

    Returns True when /health answers within ``wait_seconds``.
    """
    if is_daemon_reachable():
        return True

    try:
        start_daemon(host=host, port=port)
    except OSError as exc:
        # Another process may already own the port — re-check health.
        logging.warning("hermes daemon bind failed (%s); rechecking health", exc)
        return is_daemon_reachable()

    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        if is_daemon_reachable():
            return True
        time.sleep(_STARTUP_POLL_SECONDS)
    return is_daemon_reachable()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s hermes-daemon: %(message)s",
    )
    serve_forever()
