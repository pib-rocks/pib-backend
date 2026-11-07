"""pib.Cloud web search client for live and direct chat tool calls."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

_TOKEN_FILE = Path("/home/pib/.talk-cloud-token")


def _resolve_search_url() -> str:
    base = os.environ.get("PIB_CLOUD_BASE_URL", "https://api.pib.rocks/v1").strip()
    base = base.rstrip("/")
    if base.endswith("/v1"):
        return f"{base}/search"
    return f"{base}/v1/search"


def _resolve_bearer_token() -> str:
    token = os.environ.get("PIB_CLOUD_TOKEN", "").strip()
    if token:
        return token
    try:
        if _TOKEN_FILE.is_file():
            return _TOKEN_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        pass
    return ""


def search_cloud(
    query: str,
    *,
    language: str = "de",
    max_results: int = 5,
    timeout_s: float = 12.0,
) -> dict[str, Any]:
    query = (query or "").strip()
    if not query:
        return {"ok": False, "reason": "query_empty"}
    if len(query) > 500:
        return {"ok": False, "reason": "query_too_long"}

    token = _resolve_bearer_token()
    if not token:
        return {"ok": False, "reason": "missing_cloud_token"}

    url = _resolve_search_url()
    body = json.dumps(
        {"query": query, "language": language, "max_results": max_results}
    ).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "Accept": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            data = json.loads(response.read().decode("utf-8"))
        return {
            "ok": True,
            "query": data.get("query", query),
            "answer": data.get("answer", ""),
            "sources": data.get("sources", []),
        }
    except urllib.error.HTTPError as exc:
        return {"ok": False, "reason": f"http_{exc.code}"}
    except Exception:
        return {"ok": False, "reason": "search_unavailable"}
