#!/usr/bin/env python3
"""Registry pull vs local build for pib-backend compose services (PR-2084).

Dependency-free: imported and executed on the robot host next to update_runner.sh.
"""

from __future__ import annotations

import os
import sys
from typing import Final

# Services that CI publishes and compose names with ghcr.io/pib-rocks/pib-backend-<service>.
BACKEND_REGISTRY_SERVICES: Final[tuple[str, ...]] = (
    "hermes-daemon",
    "pib-blockly-server",
    "flask-app",
    "rosbridge-ws",
    "ros-camera",
    "ros-motors",
    "ros-voice-assistant",
    "ros-programs",
    "ros-display",
    "ros-audio-io",
)

# (compose service name, build context relative to repo root, dockerfile relative to repo root)
BACKEND_IMAGE_BUILD_SPECS: Final[tuple[tuple[str, str, str], ...]] = (
    ("hermes-daemon", ".", "public_api_client/Dockerfile"),
    ("pib-blockly-server", "pib_blockly/pib_blockly_server", "Dockerfile"),
    ("flask-app", ".", "pib_api/flask/Dockerfile"),
    ("rosbridge-ws", "ros_packages", "rosbridge/Dockerfile"),
    ("ros-camera", "ros_packages", "camera/Dockerfile"),
    ("ros-motors", ".", "ros_packages/motors/Dockerfile"),
    ("ros-voice-assistant", ".", "ros_packages/voice_assistant/Dockerfile"),
    ("ros-programs", ".", "ros_packages/programs/Dockerfile"),
    ("ros-display", ".", "ros_packages/display/Dockerfile"),
    ("ros-audio-io", ".", "ros_packages/ros_audio_io/Dockerfile"),
)

DEFAULT_IMAGE_PREFIX: Final[str] = "ghcr.io/pib-rocks/pib-backend"
DEFAULT_IMAGE_TAG: Final[str] = "develop"

LOG_PULL_SUCCESS: Final[str] = (
    "Backend service images: pulled from registry (recreate with --no-build)"
)
LOG_BUILD_NO_REGISTRY: Final[str] = (
    "Backend service images: local build (registry pull disabled)"
)
LOG_BUILD_AFTER_PULL_FAILED: Final[str] = (
    "Backend service images: local build (registry pull failed; compose build fallback)"
)


def registry_pull_enabled(
    environ: os._Environ[str] | dict[str, str] | None = None,
) -> bool:
    """False when the operator opts out of ghcr.io pulls (offline or unpublished images)."""
    env = os.environ if environ is None else environ
    raw = env.get("PIB_USE_REGISTRY_IMAGES", "1").strip().lower()
    return raw not in {"0", "false", "no", "off"}


def resolve_image_tag(
    app_version: str | None,
    environ: os._Environ[str] | dict[str, str] | None = None,
) -> str:
    """Tag appended to ghcr.io/.../pib-backend-<service> (matches CI channel/version tags)."""
    env = os.environ if environ is None else environ
    explicit = env.get("PIB_IMAGE_TAG", "").strip()
    if explicit:
        return explicit
    if app_version and app_version.strip():
        return app_version.strip()
    return DEFAULT_IMAGE_TAG


def deploy_strategy(
    *,
    registry_enabled: bool,
    pull_succeeded: bool | None,
) -> str:
    """Return ``pull``, ``build``, or ``build_after_pull_failed``."""
    if not registry_enabled:
        return "build"
    if pull_succeeded is True:
        return "pull"
    if pull_succeeded is False:
        return "build_after_pull_failed"
    return "build"


def log_summary_for_strategy(strategy: str) -> str:
    if strategy == "pull":
        return LOG_PULL_SUCCESS
    if strategy == "build_after_pull_failed":
        return LOG_BUILD_AFTER_PULL_FAILED
    return LOG_BUILD_NO_REGISTRY


def image_reference(service: str, prefix: str, tag: str) -> str:
    return f"{prefix}-{service}:{tag}"


def _usage() -> None:
    sys.stderr.write(
        "usage: update_backend_images.py registry-enabled | resolve-tag APP_VERSION\n"
        "       update_backend_images.py log-summary STRATEGY\n"
    )


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        _usage()
        return 2
    command = args[0]
    if command == "registry-enabled":
        print("true" if registry_pull_enabled() else "false")
        return 0
    if command == "resolve-tag":
        app_version = args[1] if len(args) > 1 else ""
        print(resolve_image_tag(app_version or None))
        return 0
    if command == "log-summary":
        if len(args) < 2:
            _usage()
            return 2
        print(log_summary_for_strategy(args[1]))
        return 0
    _usage()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
