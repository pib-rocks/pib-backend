"""Shared voice ids for cloud speech and local Supertone on Edu and Pro."""

from __future__ import annotations

# Cloud speech endpoint voice names (Domi / Adam).
VOICE_FEMALE_CLOUD = "F2"
VOICE_MALE_CLOUD = "M3"


def resolve_voice_id(gender: object) -> str:
    """Map the personality gender label to the cloud speech ``voice`` field."""
    text = gender.strip().lower() if isinstance(gender, str) else ""
    return VOICE_MALE_CLOUD if text.startswith("m") else VOICE_FEMALE_CLOUD
