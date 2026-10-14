"""Confirm a named live model against that account's model list.

Gemini is ``GET /v1beta/models``. An OpenAI-compatible account is
``GET /v1/models``. The check is recorded on the live model row itself.
A chat model is left alone: the list must not hide a different id on it.
A failed read leaves the row as it was.
"""

from __future__ import annotations

import json
import logging
from datetime import date
from typing import Iterable, Optional
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from app.app import db
from model.provider_model import RegistryModel
from pib_hermes_config.live_session import (
    GEMINI_LIVE_MODEL,
    GEMINI_MODELS_URL,
    OPENAI_LIVE_MODEL,
    gemini_live_connect_model,
    openai_models_url,
)

logger = logging.getLogger(__name__)

_MAX_PAGES = 10


def _as_capabilities(raw: object) -> dict:
    if isinstance(raw, str):
        loaded = json.loads(raw)
        raw = loaded
    if isinstance(raw, dict):
        return dict(raw)
    return {}


def _list_kind(api_name: str) -> Optional[str]:
    """Which account list this id is read from. The id is not rewritten."""
    name = (api_name or "").lower()
    if "gemini" in name:
        return "gemini"
    if name.startswith("gpt-"):
        return "openai"
    return None


def apply_live_pin(
    model: RegistryModel, model_ids: Iterable[str], checked_on: date
) -> None:
    """Record one successful list read on a named live model.

    The row's own id is the live model. When that id is absent the live
    flag is cleared so a retired preview cannot be used instead. A chat
    model is not given another id. ``gpt-realtime`` keeps the live flag
    off: this process has no OpenAI realtime transport.
    """
    if model.api_name not in (GEMINI_LIVE_MODEL, OPENAI_LIVE_MODEL):
        return
    present = model.api_name in set(model_ids)
    model.live_model = None
    model.live_model_checked_on = checked_on
    capabilities = _as_capabilities(model.capabilities)
    if model.api_name == GEMINI_LIVE_MODEL:
        capabilities["live"] = bool(
            present and gemini_live_connect_model(model.api_name)
        )
    else:
        capabilities["live"] = False
    model.capabilities = capabilities


def ids_from_list_payload(
    payload: object, kind: str
) -> tuple[list[str], Optional[str]]:
    """Bare model ids and the next page token, if the payload has one."""
    if not isinstance(payload, dict):
        return [], None
    if kind == "gemini":
        ids: list[str] = []
        for row in payload.get("models") or []:
            if not isinstance(row, dict):
                continue
            name = str(row.get("name") or "")
            bare = name.rsplit("/", 1)[-1]
            if bare:
                ids.append(bare)
        token = payload.get("nextPageToken") or None
        return ids, token if isinstance(token, str) and token else None
    ids = []
    for row in payload.get("data") or []:
        if isinstance(row, dict) and row.get("id"):
            ids.append(str(row["id"]))
    return ids, None


def _request_for(
    api_name: str, endpoint_base: object, api_key: str, page_token: Optional[str]
) -> Optional[Request]:
    kind = _list_kind(api_name)
    if kind == "gemini":
        params = {"pageSize": "100"}
        if page_token:
            params["pageToken"] = page_token
        url = GEMINI_MODELS_URL + "?" + urlencode(params)
        return Request(url, headers={"x-goog-api-key": api_key})
    if kind == "openai":
        return Request(
            openai_models_url(endpoint_base),
            headers={"Authorization": "Bearer " + api_key},
        )
    return None


def fetch_model_ids(
    api_name: str,
    endpoint_base: object,
    api_key: str,
    opener=None,
    timeout: float = 10.0,
) -> list[str]:
    """Read one account's model list. The key is a header, never logged."""
    if opener is None:
        opener = urlopen
    kind = _list_kind(api_name) or "openai"
    found: list[str] = []
    page_token: Optional[str] = None
    for _ in range(_MAX_PAGES):
        request = _request_for(api_name, endpoint_base, api_key, page_token)
        if request is None:
            return []
        with opener(request, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
        ids, page_token = ids_from_list_payload(payload, kind)
        found.extend(ids)
        if not page_token:
            break
    return found


def pin_providers(
    rows: Iterable[RegistryModel],
    secrets: dict[str, str],
    fetch=None,
    checked_on: Optional[date] = None,
) -> None:
    """Confirm each named live model whose provider secret can read a list."""
    if fetch is None:
        fetch = fetch_model_ids
    day = checked_on or date.today()
    for model in rows:
        provider = model.provider
        if provider is None:
            continue
        ref = provider.credential_ref
        if not ref or ref not in secrets:
            continue
        if model.api_name not in (GEMINI_LIVE_MODEL, OPENAI_LIVE_MODEL):
            continue
        secret = secrets[ref]
        if not isinstance(secret, str) or not secret.strip():
            continue
        try:
            model_ids = fetch(model.api_name, provider.endpoint_base, secret)
        except Exception:
            logger.warning(
                "Live model list could not be read for provider %s.",
                provider.id,
                exc_info=True,
            )
            continue
        apply_live_pin(model, model_ids, day)
    db.session.flush()


def pin_unlocked_providers() -> None:
    """Re-pin from the keys this process is holding. No-op while locked."""
    from service.key_store_service import unlocked_credentials

    secrets = unlocked_credentials()
    if not secrets:
        return
    pin_providers(RegistryModel.query.all(), secrets)
