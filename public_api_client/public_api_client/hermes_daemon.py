"""Long-lived Hermes Agent daemon for low-latency voice assistant turns.

Exposes a localhost HTTP API so ``hermes_agent_client.run_turn`` can dispatch
to a warm process instead of cold-starting the Hermes CLI on every chat turn.

Endpoints:
  GET  /health  → 200 {"status": "ok"}
  POST /turn    → JSON {text, chat_id, personality_id?, toolsets?} → {"reply": "..."}
"""
from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time

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


def _default_turn_runner(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: Optional[int] = None,
) -> str:
    """Execute one turn via the Hermes CLI subprocess (no daemon recursion)."""
    from public_api_client.hermes_agent_client import run_turn_subprocess

    kwargs = {
        "text": text,
        "chat_id": chat_id,
        "personality_id": personality_id,
        "toolsets": toolsets,
    }
    if timeout is not None:
        kwargs["timeout"] = timeout
    return run_turn_subprocess(**kwargs)


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
        timeout = data.get("timeout")
        if personality_id is not None and not isinstance(personality_id, str):
            self._send_json(400, {"error": "personality_id must be a string"})
            return
        if toolsets is not None and not isinstance(toolsets, str):
            self._send_json(400, {"error": "toolsets must be a string"})
            return
        if timeout is not None and not isinstance(timeout, (int, float)):
            self._send_json(400, {"error": "timeout must be a number"})
            return

        runner = getattr(self.server, "turn_runner", None) or _default_turn_runner
        turn_start = time.monotonic()
        logging.info(
            "[PERF_TRACE] DAEMON_TURN_START chat=%s elapsed_ms=%.2f",
            chat_id, (turn_start - t0) * 1000.0,
        )
        try:
            reply = runner(
                text=text,
                chat_id=chat_id,
                personality_id=personality_id,
                toolsets=toolsets,
                timeout=int(timeout) if timeout is not None else None,
            )
        except Exception as exc:
            logging.exception("hermes-daemon /turn failed: %s", exc)
            self._send_json(500, {"error": str(exc)})
            return

        # Full reply is available at once from the runner; treat that as first token.
        logging.info(
            "[PERF_TRACE] DAEMON_FIRST_TOKEN chat=%s elapsed_ms=%.2f",
            chat_id, (time.monotonic() - t0) * 1000.0,
        )
        logging.info(
            "[PERF_TRACE] DAEMON_DONE chat=%s elapsed_ms=%.2f",
            chat_id, (time.monotonic() - t0) * 1000.0,
        )
        self._send_json(200, {"reply": reply if isinstance(reply, str) else str(reply)})


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
        if _server is not None and _server_thread is not None and _server_thread.is_alive():
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
