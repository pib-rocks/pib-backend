"""Pair published pib-backend and cerebra releases.

This is the host-side companion of ``.github/scripts/verify_release_pairing.py``
and ``docs/RELEASE.md``. Both use the same repositories, the same draft rule,
and the same stable tag. The tag is published on ``develop`` and may only
appear on ``main`` after the ``--no-ff`` merge, so a tag is resolved to the
commit it points at rather than to ``origin/main``.

Discovery never accepts a repository or ref supplied by a client. Prereleases
and non-semver tags, including model-registry assets, are not system releases.
"""

from __future__ import annotations

import json
import os
import re
import sys
import urllib.error
import urllib.request
from typing import Any, Callable, Mapping

BACKEND_REPO = "pib-rocks/pib-backend"
CEREBRA_REPO = "pib-rocks/cerebra"
KNOWN_REPOS = frozenset({BACKEND_REPO, CEREBRA_REPO})
API = "https://api.github.com"
STABLE_TAG = re.compile(r"^v(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$")
SHA = re.compile(r"^[0-9a-f]{40}$")
MAX_PAGES = 3
NOTE_LIMIT = 2000

Transport = Callable[[str], tuple[int, object]]


def version_tuple(tag: object) -> tuple[int, int, int] | None:
    if not isinstance(tag, str):
        return None
    match = STABLE_TAG.fullmatch(tag.strip())
    if match is None:
        return None
    return tuple(int(part) for part in match.groups())


def _stable_tag(release: Mapping[str, Any]) -> str | None:
    tag = release.get("tag_name")
    if isinstance(tag, str) and STABLE_TAG.fullmatch(tag):
        return tag
    return None


def _index(releases: object) -> dict[str, Mapping[str, Any]]:
    indexed: dict[str, Mapping[str, Any]] = {}
    if not isinstance(releases, list):
        return indexed
    for release in releases:
        if isinstance(release, Mapping):
            tag = release.get("tag_name")
            if isinstance(tag, str) and tag not in indexed:
                indexed[tag] = release
    return indexed


def _notes(release: Mapping[str, Any] | None) -> str:
    if not isinstance(release, Mapping):
        return ""
    body = release.get("body")
    if not isinstance(body, str):
        return ""
    return body.strip()[:NOTE_LIMIT]


def _commit(
    commits: Mapping[str, Mapping[str, object]], repo: str, tag: str
) -> str | None:
    value = (commits.get(repo) or {}).get(tag)
    if isinstance(value, str) and SHA.fullmatch(value):
        return value
    return None


def pair_releases(
    backend_releases: object,
    cerebra_releases: object,
    commits: Mapping[str, Mapping[str, object]] | None = None,
) -> dict[str, Any]:
    """Classify GitHub release documents. ``commits`` maps repo to tag to SHA."""
    commits = commits or {}
    backend = _index(backend_releases)
    cerebra = _index(cerebra_releases)
    paired: list[dict[str, Any]] = []
    incomplete: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    seen: set[str] = set()

    for tag in list(backend) + list(cerebra):
        if tag in seen:
            continue
        seen.add(tag)
        backend_release = backend.get(tag)
        cerebra_release = cerebra.get(tag)
        sample = backend_release or cerebra_release or {}
        if version_tuple(tag) is None:
            excluded.append(
                {
                    "tag": tag,
                    "installable": False,
                    "reason": "not a stable system release tag",
                }
            )
            continue
        if any(
            isinstance(item, Mapping) and item.get("prerelease") is True
            for item in (backend_release, cerebra_release)
        ):
            excluded.append(
                {
                    "tag": tag,
                    "installable": False,
                    "reason": "prerelease is not a system release",
                }
            )
            continue
        if any(
            isinstance(item, Mapping) and item.get("draft") is True
            for item in (backend_release, cerebra_release)
        ):
            incomplete.append(
                {
                    "tag": tag,
                    "installable": False,
                    "missing": [
                        name
                        for name, item in (
                            ("pib-backend", backend_release),
                            ("cerebra", cerebra_release),
                        )
                        if not isinstance(item, Mapping) or item.get("draft") is True
                    ],
                    "reason": "release is still a draft and creates no installable tag",
                }
            )
            continue
        missing = [
            name
            for name, item in (
                ("pib-backend", backend_release),
                ("cerebra", cerebra_release),
            )
            if item is None
        ]
        if missing:
            incomplete.append(
                {
                    "tag": tag,
                    "installable": False,
                    "missing": missing,
                    "reason": "published on only one of the paired repositories",
                }
            )
            continue
        backend_commit = _commit(commits, BACKEND_REPO, tag)
        cerebra_commit = _commit(commits, CEREBRA_REPO, tag)
        if backend_commit is None or cerebra_commit is None:
            incomplete.append(
                {
                    "tag": tag,
                    "installable": False,
                    "missing": [
                        name
                        for name, commit in (
                            ("pib-backend", backend_commit),
                            ("cerebra", cerebra_commit),
                        )
                        if commit is None
                    ],
                    "reason": "published tag commit could not be resolved",
                }
            )
            continue
        published_at = None
        if isinstance(backend_release, Mapping):
            published = backend_release.get("published_at")
            if isinstance(published, str):
                published_at = published
        paired.append(
            {
                "tag": tag,
                "installable": True,
                "notes": _notes(backend_release) or _notes(cerebra_release),
                "publishedAt": published_at,
                "targets": {
                    "pib-backend": {"commit": backend_commit, "tag": tag},
                    "cerebra": {"commit": cerebra_commit, "tag": tag},
                },
            }
        )

    paired.sort(key=lambda item: version_tuple(item["tag"]) or (0, 0, 0))
    latest = paired[-1]["tag"] if paired else None
    return {
        "releases": paired,
        "incomplete": incomplete,
        "excluded": excluded,
        "latestInstallable": latest,
        "error": None,
    }


def device_channel(repositories: object) -> str:
    """Persisted checkout channel. Unknown is not rewritten as release."""
    if not isinstance(repositories, Mapping):
        return "unknown"
    channels: list[str] = []
    for name in ("pib-backend", "cerebra"):
        entry = repositories.get(name)
        if not isinstance(entry, Mapping):
            continue
        channel = entry.get("channel")
        if (
            isinstance(channel, str)
            and channel.strip()
            and channel.strip() != "unknown"
        ):
            channels.append(channel.strip())
    if not channels:
        return "unknown"
    if len(set(channels)) == 1:
        return channels[0]
    return "mixed"


def _shas_match(targets: object, repositories: object) -> bool:
    if not isinstance(targets, Mapping) or not isinstance(repositories, Mapping):
        return False
    for name in ("pib-backend", "cerebra"):
        target = targets.get(name)
        commit = target.get("commit") if isinstance(target, Mapping) else target
        installed = repositories.get(name)
        sha = installed.get("gitSha") if isinstance(installed, Mapping) else None
        if not isinstance(commit, str) or not isinstance(sha, str):
            return False
        if commit.lower() != sha.lower():
            return False
    return True


def relation_for(tag: str, targets: object, installed: Mapping[str, Any]) -> str:
    """How a paired release relates to what this device reports as installed."""
    repositories = installed.get("repositories")
    image = installed.get("imageVersion")
    matched = _shas_match(targets, repositories)
    channel = device_channel(repositories)
    image_text = image.strip() if isinstance(image, str) else ""
    if channel == "develop" or image_text == "develop":
        return "current" if matched else "channel-change"
    if not image_text or image_text == "unknown":
        return "current" if matched else "unknown"
    current = version_tuple(image_text)
    target = version_tuple(tag)
    if current is None or target is None:
        return "current" if matched else "unknown"
    if current == target:
        return "current" if matched else "drift"
    if matched:
        return "current"
    if target > current:
        return "newer"
    return "older"


def annotate_relations(
    document: Mapping[str, Any], installed: Mapping[str, Any]
) -> dict[str, Any]:
    """Add installed-version relations without treating a branch SHA as a release."""
    copied = dict(document)
    releases: list[dict[str, Any]] = []
    highlighted = None
    latest = copied.get("latestInstallable")
    for item in copied.get("releases") or []:
        if not isinstance(item, dict):
            continue
        entry = dict(item)
        tag = entry.get("tag")
        entry["relation"] = (
            relation_for(tag, entry.get("targets"), installed)
            if isinstance(tag, str)
            else "unknown"
        )
        releases.append(entry)
        if tag == latest:
            highlighted = entry
    copied["releases"] = releases
    copied["installedVersion"] = (
        installed.get("imageVersion")
        if isinstance(installed.get("imageVersion"), str)
        and installed.get("imageVersion")
        else "unknown"
    )
    copied["deviceChannel"] = device_channel(installed.get("repositories"))
    if highlighted is None:
        copied["recommendation"] = None
    else:
        relation = highlighted.get("relation")
        copied["recommendation"] = {
            "tag": highlighted.get("tag"),
            "relation": relation,
            "installable": highlighted.get("installable") is True,
            "ordinaryUpdate": relation == "newer",
            "notes": highlighted.get("notes") or "",
            "targets": highlighted.get("targets"),
        }
    if copied.get("channel") == "release":
        repositories = copied.get("repositories")
        if isinstance(repositories, dict):
            adjusted = {}
            for name, entry in repositories.items():
                if not isinstance(entry, dict):
                    adjusted[name] = entry
                    continue
                repo_entry = dict(entry)
                relation = highlighted.get("relation") if highlighted else None
                if relation == "newer":
                    target = ((highlighted or {}).get("targets") or {}).get(name)
                    commit = (
                        target.get("commit") if isinstance(target, Mapping) else None
                    )
                    installed_sha = entry.get("installed")
                    repo_entry["updateAvailable"] = (
                        True
                        if isinstance(commit, str)
                        and isinstance(installed_sha, str)
                        and commit != installed_sha
                        else "unknown"
                    )
                elif relation == "current":
                    repo_entry["updateAvailable"] = False
                else:
                    repo_entry["updateAvailable"] = "unknown"
                adjusted[name] = repo_entry
            copied["repositories"] = adjusted
    return copied


def _default_transport(url: str) -> tuple[int, object]:
    if not url.startswith(f"{API}/repos/"):
        raise ValueError("refusing a URL outside the GitHub repository API")
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "pib-update-release-check",
    }
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
        except (OSError, UnicodeError, json.JSONDecodeError):
            payload = {}
        return exc.code, payload
    except urllib.error.URLError as exc:
        raise OSError(str(exc)) from exc


def _require_repo(repo: str) -> None:
    if repo not in KNOWN_REPOS:
        raise ValueError(f"unsupported repository: {repo}")


def list_releases(repo: str, transport: Transport) -> list[object]:
    _require_repo(repo)
    collected: list[object] = []
    for page in range(1, MAX_PAGES + 1):
        status, payload = transport(
            f"{API}/repos/{repo}/releases?per_page=100&page={page}"
        )
        if status != 200 or not isinstance(payload, list):
            raise OSError(f"{repo} release list returned HTTP {status}")
        collected.extend(payload)
        if len(payload) < 100:
            break
    return collected


def resolve_commit(repo: str, tag: str, transport: Transport) -> str | None:
    _require_repo(repo)
    if version_tuple(tag) is None:
        raise ValueError(
            "refusing to resolve a tag that is not a stable system release"
        )
    status, payload = transport(f"{API}/repos/{repo}/commits/{tag}")
    if status != 200 or not isinstance(payload, dict):
        return None
    sha = payload.get("sha")
    if isinstance(sha, str) and SHA.fullmatch(sha):
        return sha
    return None


def discover(transport: Transport | None = None) -> dict[str, Any]:
    """Resolve paired stable releases. Network failures stay in ``error``."""
    transport = transport or _default_transport
    try:
        backend = list_releases(BACKEND_REPO, transport)
        cerebra = list_releases(CEREBRA_REPO, transport)
        paired_preview = pair_releases(backend, cerebra, {})
        tags = [
            item["tag"]
            for item in paired_preview["releases"] + paired_preview["incomplete"]
            if isinstance(item.get("tag"), str) and version_tuple(item["tag"])
        ]
        commits: dict[str, dict[str, str | None]] = {
            BACKEND_REPO: {},
            CEREBRA_REPO: {},
        }
        for repo in (BACKEND_REPO, CEREBRA_REPO):
            for tag in tags:
                commits[repo][tag] = resolve_commit(repo, tag, transport)
        return pair_releases(backend, cerebra, commits)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        return {
            "releases": [],
            "incomplete": [],
            "excluded": [],
            "latestInstallable": None,
            "error": f"Release discovery failed: {error}",
        }


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) == 2 and arguments[0] == "discover":
        document = discover()
        destination = arguments[1]
        temporary = destination + ".tmp"
        with open(temporary, "w", encoding="utf-8") as output:
            json.dump(document, output, sort_keys=True)
            output.write("\n")
        os.replace(temporary, destination)
        return 0
    sys.stderr.write("usage: update_releases.py discover OUTPUT_FILE\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
