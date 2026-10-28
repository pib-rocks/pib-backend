"""Release pairing guard decisions for CI and local checks.

Companion of ``.github/workflows/release-pairing-guard.yml`` and
``.github/scripts/verify_release_pairing.py``. Uses the same draft and
prerelease rules as ``update_releases.pair_releases``.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

_SETUP_DIR = Path(__file__).resolve().parent
if str(_SETUP_DIR) not in sys.path:
    sys.path.insert(0, str(_SETUP_DIR))

import update_releases  # noqa: E402

BACKEND_REPO = update_releases.BACKEND_REPO
CEREBRA_REPO = update_releases.CEREBRA_REPO
PIB_SDK_REPO = "pib-rocks/pib-sdk"
PIB_SDK_VERSION_FILE = "pib_sdk_version.txt"
API_TAG = "https://api.github.com/repos/{repo}/releases/tags/{tag}"
API_RELEASES = "https://api.github.com/repos/{repo}/releases?per_page=100&page=1"

Outcome = str  # OK | PENDING | ERROR | WARNING


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def normalize_sdk_version(raw: str) -> str:
    text = raw.strip()
    if text.startswith("v"):
        text = text[1:]
    return text


def sdk_tag_for_version(version: str) -> str:
    normalized = normalize_sdk_version(version)
    tag = f"v{normalized}"
    if update_releases.version_tuple(tag) is None:
        raise ValueError(
            f"invalid pib-sdk version in {PIB_SDK_VERSION_FILE}: {normalized!r}"
        )
    return tag


def read_required_sdk_version(repo_root: Path | None = None) -> str:
    """Read the single-line version from ``pib_sdk_version.txt`` at repo root."""
    root = repo_root or _repo_root()
    path = root / PIB_SDK_VERSION_FILE
    if not path.is_file():
        raise FileNotFoundError(
            f"required {PIB_SDK_VERSION_FILE} is missing at repository root ({root})"
        )
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"{PIB_SDK_VERSION_FILE} is empty")
    return normalize_sdk_version(text)


def classify_sdk_from_releases(
    releases: object, required_version: str
) -> tuple[str, str]:
    """Return (classification, tag) for the required pib-sdk version."""
    tag = sdk_tag_for_version(required_version)
    release = update_releases.release_by_tag(releases, tag)
    return update_releases.classify_github_release(release), tag


def decide_sdk_guard(
    *,
    classification: str,
    required_version: str,
    backend_age_minutes: int,
    grace_minutes: int,
) -> tuple[Outcome, list[str]]:
    """Decide the pib-sdk leg of the pairing guard."""
    tag = sdk_tag_for_version(required_version)
    source = f"{PIB_SDK_VERSION_FILE} (requires pib-sdk {required_version} / {tag})"
    lines: list[str] = []

    if classification == "published":
        lines.append(
            f"OK: pib-sdk {tag} is published ({source}). "
            "Installers can resolve this release."
        )
        return "OK", lines

    if classification in ("draft", "prerelease"):
        state = (
            f"pib-sdk {tag} exists but is still a DRAFT"
            if classification == "draft"
            else f"pib-sdk {tag} is a prerelease, not a published system release"
        )
        lines.append(f"::error::Release pair INCOMPLETE: {state}.")
        lines.append(
            f"::error::pib-backend requires {tag} from {PIB_SDK_VERSION_FILE} "
            f"({required_version}). Publish {tag} in {PIB_SDK_REPO} "
            "(not a draft or prerelease), or change the pinned version."
        )
        return "ERROR", lines

    if classification == "unstable_tag":
        lines.append(f"::error::pib-sdk release for {tag} is not a stable semver tag.")
        lines.append(f"::error::Required version came from {source}.")
        return "ERROR", lines

    # missing
    state = f"pib-sdk has no published release {tag} yet"
    if backend_age_minutes < grace_minutes:
        lines.append(
            f"::notice::PENDING - {state}. Required version {required_version} "
            f"from {PIB_SDK_VERSION_FILE}. "
            "Release order: pib-sdk first, then pib-backend, then cerebra. "
            f"Grace period {grace_minutes} min, elapsed {backend_age_minutes} min."
        )
        return "PENDING", lines

    lines.append(
        f"::error::Release pair INCOMPLETE: {state}, "
        f"{backend_age_minutes} minutes after pib-backend was published."
    )
    lines.append(
        f"::error::Required pib-sdk version {required_version} ({tag}) "
        f"from {PIB_SDK_VERSION_FILE}. Publish {tag} in {PIB_SDK_REPO}."
    )
    return "ERROR", lines


def decide_cerebra_guard(
    *,
    classification: str,
    tag: str,
    backend_age_minutes: int,
    grace_minutes: int,
) -> tuple[Outcome, list[str]]:
    """Decide the cerebra leg (grace applies to missing and draft)."""
    lines: list[str] = []

    if classification == "published":
        lines.append(f"OK: cerebra {tag} is published. The release pair is complete.")
        return "OK", lines

    if classification == "missing":
        state = f"cerebra has no release {tag} yet"
    elif classification == "draft":
        state = f"cerebra {tag} exists but is still a DRAFT"
    elif classification == "prerelease":
        state = f"cerebra {tag} is a prerelease, not a published system release"
    else:
        state = f"cerebra {tag} is not a stable published release"

    if classification in ("missing", "draft") and backend_age_minutes < grace_minutes:
        lines.append(
            f"::notice::PENDING - {state}. This is expected right after a "
            "pib-backend release (order: backend first, cerebra second). "
            f"Grace period {grace_minutes} min, elapsed {backend_age_minutes} min."
        )
        return "PENDING", lines

    lines.append(
        f"::error::Release pair INCOMPLETE: {state}, {backend_age_minutes} minutes after"
    )
    lines.append(f"::error::pib-backend {tag} was published.")
    lines.append("::error::pib-backend and cerebra must carry the same version and be")
    lines.append(f"::error::released together. Publish {tag} in {CEREBRA_REPO} and")
    lines.append("::error::merge develop -> main there, or withdraw this release.")
    return "ERROR", lines


def worst_outcome(*outcomes: Outcome) -> Outcome:
    order = {"OK": 0, "WARNING": 1, "PENDING": 2, "ERROR": 3}
    return max(outcomes, key=lambda item: order.get(item, 0))


def exit_code_for(*outcomes: Outcome) -> int:
    return 1 if any(item == "ERROR" for item in outcomes) else 0


def _get(url: str) -> tuple[int, dict[str, Any]]:
    headers = {"Accept": "application/vnd.github+json"}
    token = os.environ.get("GH_TOKEN", "").strip()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            payload = json.load(response)
            return response.status, payload if isinstance(payload, dict) else {}
    except urllib.error.HTTPError as exc:
        try:
            payload = json.load(exc)
        except Exception:
            payload = {}
        return exc.code, payload if isinstance(payload, dict) else {}
    except urllib.error.URLError as exc:
        print(f"::warning::Network error querying {url}: {exc}")
        return 0, {}


def _list_releases(repo: str) -> list[object]:
    code, payload = _get(API_RELEASES.format(repo=repo))
    if code != 200 or not isinstance(payload, list):
        return []
    return payload


def _age_minutes(published_at: str | None) -> int:
    if not published_at:
        return 0
    published = datetime.datetime.strptime(published_at, "%Y-%m-%dT%H:%M:%SZ").replace(
        tzinfo=datetime.timezone.utc
    )
    now = datetime.datetime.now(datetime.timezone.utc)
    return int((now - published).total_seconds() // 60)


def run_verify(tag: str, grace_minutes: int, repo_root: Path | None = None) -> int:
    root = repo_root or _repo_root()
    print(f"Verifying release pairing for {tag}")

    backend_code, backend = _get(API_TAG.format(repo=BACKEND_REPO, tag=tag))
    if backend_code != 200:
        print(
            f"::warning::pib-backend has no published release {tag} "
            f"(HTTP {backend_code}) - nothing to pair."
        )
        return 0

    age = _age_minutes(backend.get("published_at"))
    print(f"pib-backend {tag} was published {age} minute(s) ago.")

    outcomes: list[Outcome] = []

    try:
        required_sdk = read_required_sdk_version(root)
    except (FileNotFoundError, ValueError) as error:
        print(f"::error::{error}")
        return 1

    sdk_releases = _list_releases(PIB_SDK_REPO)
    sdk_class, _sdk_tag = classify_sdk_from_releases(sdk_releases, required_sdk)
    sdk_outcome, sdk_lines = decide_sdk_guard(
        classification=sdk_class,
        required_version=required_sdk,
        backend_age_minutes=age,
        grace_minutes=grace_minutes,
    )
    for line in sdk_lines:
        print(line)
    outcomes.append(sdk_outcome)

    cerebra_code, cerebra = _get(API_TAG.format(repo=CEREBRA_REPO, tag=tag))
    if cerebra_code == 0:
        outcomes.append("WARNING")
    elif cerebra_code == 404:
        cerebra_class = "missing"
    else:
        cerebra_class = update_releases.classify_github_release(
            cerebra if cerebra_code == 200 else None
        )
        if cerebra_code != 200:
            print(
                f"::warning::Could not query cerebra (HTTP {cerebra_code}) - "
                "will retry on the next scheduled run."
            )
            outcomes.append("WARNING")
            cerebra_class = None

    if cerebra_class is not None:
        cerebra_outcome, cerebra_lines = decide_cerebra_guard(
            classification=cerebra_class,
            tag=tag,
            backend_age_minutes=age,
            grace_minutes=grace_minutes,
        )
        for line in cerebra_lines:
            print(line)
        outcomes.append(cerebra_outcome)

    return exit_code_for(*outcomes)


def run_check_sdk(
    releases_json: Mapping[str, Any] | list[Any],
    required_version: str,
    backend_age_minutes: int,
    grace_minutes: int,
) -> int:
    releases = (
        releases_json
        if isinstance(releases_json, list)
        else releases_json.get("releases", releases_json)
    )
    classification, _tag = classify_sdk_from_releases(releases, required_version)
    outcome, lines = decide_sdk_guard(
        classification=classification,
        required_version=required_version,
        backend_age_minutes=backend_age_minutes,
        grace_minutes=grace_minutes,
    )
    for line in lines:
        print(line)
    return exit_code_for(outcome)


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description="Release pairing guard decisions")
    sub = parser.add_subparsers(dest="command", required=True)

    verify = sub.add_parser("verify", help="Live GitHub check for CI")
    verify.add_argument("tag", help="release tag to verify, e.g. v0.6.1")
    verify.add_argument("--grace-minutes", type=int, default=120)
    verify.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="repository root containing pib_sdk_version.txt",
    )

    check_sdk = sub.add_parser(
        "check-sdk", help="Evaluate pib-sdk leg from a release list JSON"
    )
    check_sdk.add_argument(
        "releases_json",
        help="path to JSON release list, or '-' for stdin",
    )
    check_sdk.add_argument("required_version", help="required semver, e.g. 0.7.4")
    check_sdk.add_argument("backend_age_minutes", type=int)
    check_sdk.add_argument("--grace-minutes", type=int, default=120)

    args = parser.parse_args(arguments)

    if args.command == "verify":
        return run_verify(args.tag, args.grace_minutes, args.repo_root)

    if args.releases_json == "-":
        payload = json.load(sys.stdin)
    else:
        with open(args.releases_json, encoding="utf-8") as handle:
            payload = json.load(handle)
    return run_check_sdk(
        payload,
        args.required_version,
        args.backend_age_minutes,
        args.grace_minutes,
    )


if __name__ == "__main__":
    raise SystemExit(main())
