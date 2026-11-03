"""In-container liveness probe for the voice stack (PR-2122).

Two facts must hold before Docker marks the container healthy:

* The chat and assistant ROS nodes are up and ``send_chat_message`` is listed.
* A provider key is resolvable for the active personality's Hermes profile route
  (same store read a turn uses via ``resolve_hermes_provider_key``).

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
    missing = [
        name
        for name in ("/chat", "/voice_assistant", "/audio_recorder")
        if name not in nodes
    ]
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


def _route_diagnostics(model: str, provider: str, profile_dir: Optional[str] = None) -> str:
    """Name the config file and credential id this store read used.

    A present key stays invisible when the route names a provider the store
    does not hold under that id - measured on 192.168.1.112, where the stale
    base config said ``gemini``/``gemini-3.8-flash`` while the key sat under
    ``pib-cloud``. Without these two facts the failure reads like a missing key.
    """
    from public_api_client.hermes_agent_client import (
        CONFIG_FILENAME,
        _current_hermes_home,
    )
    from voice_assistant.direct_tool_loop import _store_credential_id

    try:
        # Name the file the route was actually read from: the profile's config.yaml when the
        # personality resolved, the base config otherwise.
        config_path = os.path.join(profile_dir or _current_hermes_home(), CONFIG_FILENAME)
    except Exception:  # noqa: BLE001 - diagnostics must never break the probe
        config_path = "unknown"
    try:
        credential = _store_credential_id(provider, model)
    except Exception:  # noqa: BLE001
        credential = "unknown"
    return f"credential={credential} config={config_path}"


def _active_personality_profile() -> tuple[str, str] | None:
    """First backend personality whose profile directory has a config.yaml."""
    from pib_api_client import voice_assistant_client
    from pib_hermes_config import profile_dir_for
    from public_api_client.hermes_agent_client import CONFIG_FILENAME

    try:
        successful, personalities = voice_assistant_client.get_all_personalities()
    except Exception:  # noqa: BLE001 - unreachable backend must not break the probe
        return None
    if not successful:
        return None
    for personality in personalities:
        personality_id = personality.personality_id
        if not personality_id:
            continue
        profile_dir = profile_dir_for(personality_id)
        if os.path.isfile(os.path.join(profile_dir, CONFIG_FILENAME)):
            return personality_id, profile_dir
    return None


def check_provider_key() -> Tuple[bool, str, str, Optional[str]]:
    """Return ``(ok, reason, detail, provider)`` for the active profile route."""
    from pib_hermes_config import provider_needs_key
    from public_api_client.hermes_agent_client import profile_hermes_settings
    from voice_assistant.direct_tool_loop import (
        DirectToolLoopError,
        resolve_hermes_provider_key,
    )

    profile_id: Optional[str] = None
    try:
        active = _active_personality_profile()
    except Exception:  # noqa: BLE001
        active = None
    if active is not None:
        profile_id, profile_dir = active
        model, provider, _base, env_vars = profile_hermes_settings(profile_dir)
    else:
        model, provider, _base, env_vars = profile_hermes_settings()

    profile_fact = f"profile={profile_id}" if profile_id else "profile=none"
    provider_name = (provider or "").strip() or "unknown"
    if not env_vars and not provider_needs_key(provider):
        return True, "", "", provider_name

    try:
        resolve_hermes_provider_key(provider, api_name=model)
    except DirectToolLoopError as error:
        message = str(error).strip()
        route = _route_diagnostics(model, provider, profile_dir if active is not None else None)
        route_with_profile = f"{route} {profile_fact}"
        if "No keys are available for provider" in message:
            return (
                False,
                REASON_NO_KEY,
                f"{message} ({route_with_profile})",
                provider_name,
            )
        return False, REASON_KEY_CHECK, message, provider_name
    except Exception as error:  # noqa: BLE001 - report without crashing the probe
        return (
            False,
            REASON_KEY_CHECK,
            f"{type(error).__name__}: {error} "
            f"({_route_diagnostics(model, provider, profile_dir if active is not None else None)} {profile_fact})",
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
