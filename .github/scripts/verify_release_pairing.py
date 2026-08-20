#!/usr/bin/env python3
"""Verify that cerebra carries the same published release as pib-backend.

pib-backend and cerebra share ONE version number and are released as a pair.
Release order is fixed: pib-backend first, cerebra second.

Because pib-backend is released FIRST, it cannot verify cerebra up front - at
that moment cerebra legitimately has no release yet. This script therefore
reports:

  * OK       - cerebra has the same tag published                    -> exit 0
  * PENDING  - cerebra is behind, but within the grace period        -> exit 0
  * ERROR    - cerebra is still missing after the grace period       -> exit 1
  * WARNING  - the cerebra API could not be queried (retry later)    -> exit 0

The hard, build-blocking guard lives on the cerebra side
(.github/workflows/docker-build.yml): it refuses to publish a Docker image
unless the matching pib-backend release already exists.

Usage:
    verify_release_pairing.py <tag> [--grace-minutes N]

Environment:
    GH_TOKEN  optional; raises the GitHub API rate limit from 60/h to 5000/h.
              Both repositories are public, so the check works without it.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import urllib.error
import urllib.request

BACKEND_REPO = "pib-rocks/pib-backend"
CEREBRA_REPO = "pib-rocks/cerebra"
API = "https://api.github.com/repos/{repo}/releases/tags/{tag}"


def _get(url: str) -> tuple[int, dict]:
    """GET a JSON document. Returns (status_code, payload)."""
    headers = {"Accept": "application/vnd.github+json"}
    token = os.environ.get("GH_TOKEN", "").strip()
    if token:
        headers["Authorization"] = f"Bearer {token}"

    request = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as exc:
        try:
            payload = json.load(exc)
        except Exception:
            payload = {}
        return exc.code, payload
    except urllib.error.URLError as exc:
        print(f"::warning::Network error querying {url}: {exc}")
        return 0, {}


def _age_minutes(published_at: str | None) -> int:
    if not published_at:
        return 0
    published = datetime.datetime.strptime(published_at, "%Y-%m-%dT%H:%M:%SZ").replace(
        tzinfo=datetime.timezone.utc
    )
    now = datetime.datetime.now(datetime.timezone.utc)
    return int((now - published).total_seconds() // 60)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tag", help="release tag to verify, e.g. v0.6.1")
    parser.add_argument("--grace-minutes", type=int, default=120)
    args = parser.parse_args()
    tag = args.tag

    print(f"Verifying release pairing for {tag}")

    backend_code, backend = _get(API.format(repo=BACKEND_REPO, tag=tag))
    if backend_code != 200:
        print(
            f"::warning::pib-backend has no published release {tag} "
            f"(HTTP {backend_code}) - nothing to pair."
        )
        return 0

    age = _age_minutes(backend.get("published_at"))
    print(f"pib-backend {tag} was published {age} minute(s) ago.")

    cerebra_code, cerebra = _get(API.format(repo=CEREBRA_REPO, tag=tag))

    if cerebra_code == 200 and cerebra.get("draft") is False:
        print(f"OK: cerebra {tag} is published. The release pair is complete.")
        return 0

    if cerebra_code == 200:
        state = f"cerebra {tag} exists but is still a DRAFT"
    elif cerebra_code == 404:
        state = f"cerebra has no release {tag} yet"
    else:
        print(
            f"::warning::Could not query cerebra (HTTP {cerebra_code}) - "
            "will retry on the next scheduled run."
        )
        return 0

    if age < args.grace_minutes:
        print(
            f"::notice::PENDING - {state}. This is expected right after a "
            "pib-backend release (order: backend first, cerebra second). "
            f"Grace period {args.grace_minutes} min, elapsed {age} min."
        )
        return 0

    print(f"::error::Release pair INCOMPLETE: {state}, {age} minutes after")
    print(f"::error::pib-backend {tag} was published.")
    print("::error::pib-backend and cerebra must carry the same version and be")
    print(f"::error::released together. Publish {tag} in {CEREBRA_REPO} and")
    print("::error::merge develop -> main there, or withdraw this release.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
