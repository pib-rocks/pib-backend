"""Load and validate per-variant capability manifests."""

from __future__ import annotations

import json
import os
from importlib.resources import files
from typing import Any

AVAILABLE_VARIANTS: tuple[str, ...] = (
    "pib4advanced",
    "pib4edu",
    "pib5advanced",
    "pib5edu",
    "pib5museum",
)
DEFAULT_VARIANT = "pib5edu"


class UnknownVariantError(ValueError):
    """The requested variant has no bundled manifest."""

    def __init__(self, variant: str) -> None:
        self.variant = variant
        self.available = AVAILABLE_VARIANTS
        names = ", ".join(sorted(AVAILABLE_VARIANTS))
        super().__init__(f"Unknown robot variant {variant!r}; available variants: {names}")


def resolve_variant(variant: str | None = None) -> str:
    if variant is not None:
        name = variant.strip()
        if name not in AVAILABLE_VARIANTS:
            raise UnknownVariantError(name)
        return name
    from_env = (os.getenv("PIB_ROBOT_VARIANT") or "").strip()
    if from_env:
        if from_env not in AVAILABLE_VARIANTS:
            raise UnknownVariantError(from_env)
        return from_env
    return DEFAULT_VARIANT


def validate_manifest(doc: Any) -> None:
    if not isinstance(doc, dict):
        raise ValueError("manifest must be a JSON object")
    required: list[tuple[str, type | tuple[type, ...]]] = [
        ("tier", str),
        ("platform", str),
        ("sensors", list),
        ("skills", list),
        ("learned_skills", list),
        ("limits", dict),
        ("cloud", dict),
        ("degraded_without_cloud", list),
    ]
    for key, expected in required:
        if key not in doc:
            raise ValueError(f"manifest missing required key {key!r}")
        value = doc[key]
        if isinstance(expected, tuple):
            if not isinstance(value, expected):
                raise ValueError(
                    f"manifest key {key!r} must be one of "
                    f"{', '.join(t.__name__ for t in expected)}, got {type(value).__name__}"
                )
        elif not isinstance(value, expected):
            raise ValueError(
                f"manifest key {key!r} must be {expected.__name__}, "
                f"got {type(value).__name__}"
            )
    for index, skill in enumerate(doc["skills"]):
        if not isinstance(skill, str) or not skill.strip():
            raise ValueError(f"manifest skills[{index}] must be a non-empty string")
    for index, skill in enumerate(doc["learned_skills"]):
        if not isinstance(skill, str) or not skill.strip():
            raise ValueError(
                f"manifest learned_skills[{index}] must be a non-empty string"
            )
    for index, item in enumerate(doc["sensors"]):
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"manifest sensors[{index}] must be a non-empty string")
    for index, item in enumerate(doc["degraded_without_cloud"]):
        if not isinstance(item, str) or not item.strip():
            raise ValueError(
                f"manifest degraded_without_cloud[{index}] must be a non-empty string"
            )


def load_manifest(variant: str | None = None) -> dict[str, Any]:
    name = resolve_variant(variant)
    resource = files("pib_capability").joinpath("manifests", f"{name}.json")
    try:
        raw = resource.read_text(encoding="utf-8")
    except FileNotFoundError as error:
        raise UnknownVariantError(name) from error
    doc = json.loads(raw)
    validate_manifest(doc)
    return doc


def skill_names(manifest: dict[str, Any]) -> frozenset[str]:
    return frozenset(manifest["skills"])


def remove_tools_not_in_manifest(server: Any, manifest: dict[str, Any]) -> None:
    """Drop MCP tools whose names are absent from the manifest skills list."""
    allowed = skill_names(manifest)
    for tool in list(server._tool_manager.list_tools()):
        if tool.name not in allowed:
            server._tool_manager.remove_tool(tool.name)


def capability_enforcement_enabled() -> bool:
    value = os.getenv("PIB_CAPABILITY_ENFORCE")
    if value is None:
        return True
    return value.strip() != "0"
