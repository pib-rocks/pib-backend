"""Offer qwen-fast only when the Ollama tags endpoint lists it.

A failed probe is absence: the catalogue stays as it was and the request
does not fail. The row is created on the first successful sighting and kept
afterwards, so a personality that stored its id still resolves. While the
probe says the model is gone, the selection lists leave that row out.
"""

from __future__ import annotations

import json
import logging
from typing import Optional
from urllib.request import Request, urlopen

from flask import has_request_context, request

from model.assistant_model import AssistantModel
from model.provider_model import Provider, RegistryModel
from pib_hermes_config.local_model import (
    API_NAME,
    PROVIDER_NAME,
    VISUAL_NAME,
    model_capabilities,
    openai_base_url,
    provider_capabilities,
    tags_include_local_model,
    tags_url,
)

logger = logging.getLogger(__name__)

_TAGS_TIMEOUT_SECONDS = 0.4
_REQUEST_CACHE_KEY = "pib.local_model_present"


def fetch_tags() -> dict:
    """GET Ollama's tag list. Raises when the daemon cannot be read."""
    outgoing = Request(tags_url(), method="GET")
    with urlopen(outgoing, timeout=_TAGS_TIMEOUT_SECONDS) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("ollama tags response is not an object")
    return payload


def _cached_presence() -> Optional[bool]:
    if not has_request_context():
        return None
    if _REQUEST_CACHE_KEY not in request.environ:
        return None
    return bool(request.environ[_REQUEST_CACHE_KEY])


def _remember(present: bool) -> None:
    if has_request_context():
        request.environ[_REQUEST_CACHE_KEY] = present


def ensure_row() -> RegistryModel:
    """Insert or refresh the local model. Does not touch other providers."""
    from app.app import db

    row = RegistryModel.query.filter_by(api_name=API_NAME).one_or_none()
    assistant = AssistantModel.query.filter_by(api_name=API_NAME).one_or_none()
    if assistant is None:
        assistant = AssistantModel(
            api_name=API_NAME,
            visual_name=VISUAL_NAME,
            has_image_support=False,
        )
        db.session.add(assistant)
        db.session.flush()
    else:
        assistant.visual_name = VISUAL_NAME
        assistant.has_image_support = False

    provider = row.provider if row is not None else None
    if provider is None or provider.name != PROVIDER_NAME:
        provider = Provider.query.filter_by(name=PROVIDER_NAME).one_or_none()
    if provider is None:
        provider = Provider(
            name=PROVIDER_NAME,
            endpoint_base=openai_base_url(),
            credential_ref=None,
            capabilities=provider_capabilities(),
        )
        db.session.add(provider)
        db.session.flush()
    else:
        provider.endpoint_base = openai_base_url()
        provider.credential_ref = None
        provider.capabilities = provider_capabilities()

    flags = model_capabilities()
    if row is None:
        row = RegistryModel(
            id=assistant.id,
            provider_id=provider.id,
            api_name=API_NAME,
            visual_name=VISUAL_NAME,
            has_image_support=False,
            capabilities=flags,
            is_default=False,
        )
        db.session.add(row)
    else:
        row.visual_name = VISUAL_NAME
        row.has_image_support = False
        row.capabilities = flags
        row.provider_id = provider.id
    db.session.flush()
    return row


def refresh_local_model() -> bool:
    """Probe once per request and ensure the row when the tags list it.

    Any error while reading tags, or a tags document that does not name
    qwen-fast, leaves the catalogue without this model.
    """
    cached = _cached_presence()
    if cached is not None:
        return cached
    present = False
    try:
        present = tags_include_local_model(fetch_tags())
    except Exception:
        present = False
    if present:
        from app.app import db

        try:
            with db.session.begin_nested():
                ensure_row()
        except Exception:
            logger.warning(
                "Ollama lists %s but the catalogue row could not be stored.",
                API_NAME,
                exc_info=True,
            )
            present = False
    _remember(present)
    return present


def local_model_present() -> bool:
    """The probe result for this request, probing now when nothing has yet."""
    cached = _cached_presence()
    if cached is not None:
        return cached
    return refresh_local_model()


def _provider_key_configured() -> bool:
    configured = Provider.query.filter(Provider.credential_ref.isnot(None)).first()
    return configured is not None


def fallback_model() -> Optional[RegistryModel]:
    """The local model when it is listed and no provider key is configured.

    The cloud row that owns ``is_default`` is left as it is. Callers that
    resolve the ``default`` pointer use this row instead of that flag.
    """
    if not refresh_local_model():
        return None
    if _provider_key_configured():
        return None
    return RegistryModel.query.filter_by(api_name=API_NAME).one_or_none()
