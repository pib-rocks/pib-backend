#!/usr/bin/env python3
"""Verify release pairing for pib-backend (cerebra + pinned pib-sdk).

Thin wrapper around ``setup/release_pairing_guard.py`` for backwards-compatible
paths documented in ``docs/RELEASE.md``.

Usage:
    verify_release_pairing.py <tag> [--grace-minutes N]

Environment:
    GH_TOKEN  optional; raises the GitHub API rate limit from 60/h to 5000/h.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_SETUP = _ROOT / "setup"
if str(_SETUP) not in sys.path:
    sys.path.insert(0, str(_SETUP))

import release_pairing_guard  # noqa: E402


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("tag", help="release tag to verify, e.g. v0.6.1")
    parser.add_argument("--grace-minutes", type=int, default=120)
    args = parser.parse_args()
    return release_pairing_guard.run_verify(
        args.tag, args.grace_minutes, repo_root=_ROOT
    )


if __name__ == "__main__":
    raise SystemExit(main())
