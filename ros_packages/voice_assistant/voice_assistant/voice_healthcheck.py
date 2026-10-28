"""In-container liveness probe for the voice stack (PR-2122).

Two facts must hold before Docker marks the container healthy:

* The chat and assistant ROS nodes are up and ``send_chat_message`` is listed.
* A provider key is resolvable for the active Hermes profile route (same read
  a turn uses via ``provider_key_for_turn``).

Failures print ``reason=…`` (and ``provider=…`` when relevant) on stderr so
``docker inspect`` and the Flask status reader can tell the modes apart.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from typing import Optional, Tuple

REASON_CHAT_NODE = "chat_node"
REASON_CHAT_SERVICE = "chat_service"
REASON_NO_KEY = "no_key"
REASON_KEY_CHECK = "key_check"

_ROS_SHELL = (
    "source /opt/ros/jazzy/setup.bash && " "source /app/ros2_ws/install/setup.bash && "
)


def _run_ros_command(fragment: str, timeout: float) -> Tuple[int, str, str]:
    """Run one ros2 command inside the image's ROS environment."""
    completed = subprocess.run(
        ["bash", "-lc", _ROS_SHELL + fragment],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    return completed.returncode, completed.stdout or "", completed.stderr or ""


def _timeout_seconds() -> float:
    raw = os.environ.get("PIB_VOICE_HEALTHCHECK_TIMEOUT", "3").strip()
    try:
        value = float(raw)
    except ValueError:
        value = 3.0
    return value if value > 0 else 3.0


def check_ros_stack() -> Tuple[bool, str, str]:
    """Return ``(ok, reason, detail)`` for node and service presence."""
    timeout = _timeout_seconds()
    code, stdout, stderr = _run_ros_command("ros2 node list", timeout)
    if code != 0:
        detail = (stderr or stdout or "ros2 node list failed").strip()
        return False, REASON_CHAT_NODE, detail[:200]

    nodes = {line.strip() for line in stdout.splitlines() if line.strip()}
    missing = [name for name in ("/chat", "/voice_assistant") if name not in nodes]
    if missing:
        return (
            False,
            REASON_CHAT_NODE,
            f"missing nodes: {', '.join(missing)}",
        )

    code, stdout, stderr = _run_ros_command("ros2 service list", timeout)
    if code != 0:
        detail = (stderr or stdout or "ros2 service list failed").strip()
        return False, REASON_CHAT_SERVICE, detail[:200]

    services = stdout.splitlines()
    if not any("/send_chat_message" in line for line in services):
        return False, REASON_CHAT_SERVICE, "send_chat_message service not listed"
    return True, "", ""


def check_provider_key() -> Tuple[bool, str, str, Optional[str]]:
    """Return ``(ok, reason, detail, provider)`` for the active profile route."""
    from pib_hermes_config import provider_needs_key
    from public_api_client.hermes_agent_client import (
        profile_hermes_settings,
        provider_key_for_turn,
    )
    from voice_assistant.direct_tool_loop import DirectToolLoopError

    _model, provider, _base, env_vars = profile_hermes_settings()
    provider_name = (provider or "").strip() or "unknown"
    if not env_vars and not provider_needs_key(provider):
        return True, "", "", provider_name

    try:
        provider_key_for_turn()
    except DirectToolLoopError as error:
        message = str(error).strip()
        if "No keys are available for provider" in message:
            return False, REASON_NO_KEY, message, provider_name
        return False, REASON_KEY_CHECK, message, provider_name
    except Exception as error:  # noqa: BLE001 - report without crashing the probe
        return (
            False,
            REASON_KEY_CHECK,
            f"{type(error).__name__}: {error}",
            provider_name,
        )

    return True, "", "", provider_name


def _emit_failure(reason: str, detail: str, provider: Optional[str] = None) -> None:
    parts = [f"reason={reason}"]
    if provider:
        parts.append(f"provider={provider}")
    if detail:
        safe = re.sub(r"\s+", " ", detail.strip())[:240]
        parts.append(f"detail={safe}")
    print(" ".join(parts), file=sys.stderr)


def main(argv: Optional[list[str]] = None) -> int:
    _argv = argv if argv is not None else sys.argv
    if len(_argv) > 1 and _argv[1] in ("-h", "--help"):
        print(
            "voice_healthcheck: verify ROS chat stack and Hermes provider key",
            file=sys.stderr,
        )
        return 0

    ok, reason, detail = check_ros_stack()
    if not ok:
        _emit_failure(reason, detail)
        return 1

    ok, reason, detail, provider = check_provider_key()
    if not ok:
        _emit_failure(reason, detail, provider)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
