"""Machine-readable capability manifests per hardware variant."""

from .manifest import (
    UnknownVariantError,
    load_manifest,
    resolve_variant,
    skill_names,
    validate_manifest,
)

__all__ = [
    "UnknownVariantError",
    "load_manifest",
    "resolve_variant",
    "skill_names",
    "validate_manifest",
]
