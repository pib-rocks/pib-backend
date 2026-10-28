"""Read the backend version embedded in the container image."""

from __future__ import annotations

import re
from typing import Any

RELEASE_TAG = re.compile(r"^v(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$")

NON_RELEASE_VERSION_WARNING = (
    "The images were built without a release tag; this is not a release installation."
)

VERSION_FILES = (
    "/etc/pib_version",
    "/app/version.py",
)  # /etc survives the /app volume mount

REQUIRED_PIB_SDK_VERSION_FILES = ("/etc/pib_required_pib_sdk_version",)


def read_app_version() -> str:
    for path in VERSION_FILES:
        try:
            with open(path, encoding="utf-8") as version_file:
                value = version_file.read().strip().strip('"').strip("'")
            if value:
                return value
        except (OSError, UnicodeError):
            continue
    return "unknown"


def is_release_tag(version: str) -> bool:
    return RELEASE_TAG.fullmatch(version.strip()) is not None


def version_provenance(image_version: str) -> dict[str, Any]:
    release = is_release_tag(image_version)
    return {
        "imageVersion": image_version,
        "isReleaseTag": release,
        "warning": None if release else NON_RELEASE_VERSION_WARNING,
    }


def read_required_pib_sdk_version() -> str:
    for path in REQUIRED_PIB_SDK_VERSION_FILES:
        try:
            with open(path, encoding="utf-8") as version_file:
                value = version_file.read().strip()
            if value:
                return value
        except (OSError, UnicodeError):
            continue
    return "unknown"
