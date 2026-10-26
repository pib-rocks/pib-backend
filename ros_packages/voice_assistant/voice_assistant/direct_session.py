"""Per-personality Direct session state (PR-2075).

Caches the provider key for the current key-store epoch, reuses one Gemini HTTP
connection for streaming calls, and freezes the system prompt bytes for the
lifetime of the session so provider-side prompt caches can hit.
"""

from __future__ import annotations

from typing import Optional

from pib_api_client.key_store_client import read_key_store_epoch

from voice_assistant.direct_tool_loop import (
    PINNED_PROVIDER,
    resolve_provider_key,
)


class DirectSession:
    """Transport and credential state for repeated Direct turns on one personality."""

    def __init__(self, personality_id: str) -> None:
        self.personality_id = personality_id or ""
        self._key_epoch: Optional[int] = None
        self._api_key: Optional[str] = None
        self._key_source: Optional[str] = None
        self._stable_system_prompt: Optional[str] = None

    def stable_system_prompt(self, computed: str) -> str:
        """Return the first system prompt for this session; later turns reuse it."""
        if self._stable_system_prompt is None:
            self._stable_system_prompt = computed
        return self._stable_system_prompt

    def resolve_api_key(
        self,
        provider: str = PINNED_PROVIDER,
        *,
        api_name: Optional[str] = None,
    ) -> tuple[str, str]:
        """Key for this provider, fetched once per key-store epoch."""
        epoch = read_key_store_epoch()
        if (
            self._api_key is not None
            and self._key_source is not None
            and self._key_epoch == epoch
        ):
            return self._api_key, self._key_source
        api_key, source = resolve_provider_key(provider, api_name=api_name)
        self._key_epoch = epoch
        self._api_key = api_key
        self._key_source = source
        return api_key, source
