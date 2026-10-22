#!/usr/bin/env python3
"""Copy the pib-sdk motion model into pib_mcp_server/data, or verify it.

The container must not install pinocchio/numpy just to describe the robot, so the
generated artifact is checked in. Run this after a pib-sdk release:

    python tools/build_mcp_motion_model.py          # refresh
    python tools/build_mcp_motion_model.py --check  # CI drift guard
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TARGET = REPO_ROOT / "pib_mcp_server" / "data" / "motion_model.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    try:
        from pib_sdk.motion_model import build_motion_model, serialize_motion_model
    except ImportError as exc:
        print(
            "pib-sdk with pib_sdk.motion_model is required: pip install 'pib-sdk>=0.5.6'",
            file=sys.stderr,
        )
        print(exc, file=sys.stderr)
        return 2

    built = serialize_motion_model(build_motion_model())
    if args.check:
        if not TARGET.is_file():
            print(f"missing {TARGET}", file=sys.stderr)
            return 1
        if TARGET.read_text(encoding="utf-8") == built:
            return 0
        print(f"{TARGET} differs from pib_sdk's motion model", file=sys.stderr)
        return 1

    TARGET.parent.mkdir(parents=True, exist_ok=True)
    TARGET.write_text(built, encoding="utf-8")
    print(f"wrote {TARGET} ({len(built.encode('utf-8'))} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
