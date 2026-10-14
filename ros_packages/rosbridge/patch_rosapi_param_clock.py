#!/usr/bin/env python3
"""Point rosapi's parameter-client cache stamp at the node clock.

ros-jazzy-rosapi 2.7.1 is the Jazzy release in rosdistro (2.7.1-1) and the only
binary in the ROS apt pool. It caches a parameter client with
``last_used_time=Time()`` (SYSTEM_TIME). Cleanup subtracts that from the node
clock (ROS_TIME) and rclpy raises ``TypeError: Can't subtract times with
different clock types``, which kills rosapi_node.

Upstream jazzy commit 4da27bb4266c7feb2dc09e3778bec94827e07814 (backport of
RobotWebTools/rosbridge_suite#1308) measures the same lifetime with
``time.monotonic()``. That commit is still package version 2.7.1 and has not
been bloom-released, so the image cannot install it.

This script rewrites only the known constructor so the initial stamp uses the
same node clock as cleanup. It leaves that result, and the upstream monotonic
correction, unchanged. Any other source shape exits non-zero.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

BUGGY_CONSTRUCTOR = (
    "    _cached_clients[service_name] = "
    "_CachedClient(use_count=0, last_used_time=Time(), client=client)"
)
NODE_CLOCK_CONSTRUCTOR = (
    "    _cached_clients[service_name] = "
    "_CachedClient(use_count=0, last_used_time=_node.get_clock().now(), client=client)"
)
NODE_CLOCK_CLEANUP_NOW = "    now = _node.get_clock().now()"
NODE_CLOCK_CLEANUP_COMPARE = (
    "        if cached_client.use_count == 0 and "
    "(now - cached_client.last_used_time).nanoseconds > int("
)
NODE_CLOCK_REFRESH = (
    "        _cached_clients[service_name].last_used_time = _node.get_clock().now()"
)
UPSTREAM_CONSTRUCTOR = (
    "        use_count=0, last_used_time=time.monotonic(), client=client"
)
UPSTREAM_CLEANUP_NOW = "    now = time.monotonic()"
UPSTREAM_USE_COUNT = "            cached_client.use_count == 0"
UPSTREAM_CLEANUP_COMPARE = (
    "            and now - cached_client.last_used_time > _client_persistence_sec"
)
UPSTREAM_REFRESH = (
    "        _cached_clients[service_name].last_used_time = time.monotonic()"
)

SHAPE_BUGGY = "buggy-time-constructor"
SHAPE_NODE_CLOCK = "already-node-clock"
SHAPE_UPSTREAM = "already-upstream-monotonic"
ACTION_PATCHED = "patched"

_MARKERS = {
    "buggy_constructor": BUGGY_CONSTRUCTOR,
    "node_clock_constructor": NODE_CLOCK_CONSTRUCTOR,
    "node_clock_cleanup_now": NODE_CLOCK_CLEANUP_NOW,
    "node_clock_cleanup_compare": NODE_CLOCK_CLEANUP_COMPARE,
    "node_clock_refresh": NODE_CLOCK_REFRESH,
    "upstream_constructor": UPSTREAM_CONSTRUCTOR,
    "upstream_cleanup_now": UPSTREAM_CLEANUP_NOW,
    "upstream_use_count": UPSTREAM_USE_COUNT,
    "upstream_cleanup_compare": UPSTREAM_CLEANUP_COMPARE,
    "upstream_refresh": UPSTREAM_REFRESH,
}

_NODE_CLOCK_CLEANUP = {
    "node_clock_cleanup_now": 1,
    "node_clock_cleanup_compare": 1,
    "node_clock_refresh": 2,
    "upstream_constructor": 0,
    "upstream_cleanup_now": 0,
    "upstream_use_count": 0,
    "upstream_cleanup_compare": 0,
    "upstream_refresh": 0,
}
_SHAPES = {
    SHAPE_BUGGY: {
        "buggy_constructor": 1,
        "node_clock_constructor": 0,
        **_NODE_CLOCK_CLEANUP,
    },
    SHAPE_NODE_CLOCK: {
        "buggy_constructor": 0,
        "node_clock_constructor": 1,
        **_NODE_CLOCK_CLEANUP,
    },
    SHAPE_UPSTREAM: {
        "buggy_constructor": 0,
        "node_clock_constructor": 0,
        "node_clock_cleanup_now": 0,
        "node_clock_cleanup_compare": 0,
        "node_clock_refresh": 0,
        "upstream_constructor": 1,
        "upstream_cleanup_now": 1,
        "upstream_use_count": 1,
        "upstream_cleanup_compare": 1,
        "upstream_refresh": 2,
    },
}


class RosapiClockPatchError(Exception):
    """The installed params.py is not a source shape this patch may touch."""


class PatchResult:
    """Source after a patch attempt, and which accepted shape it was."""

    def __init__(self, action: str, source: str) -> None:
        self.action = action
        self.source = source


def marker_counts(source: str) -> dict[str, int]:
    """Count the exact source lines that identify each accepted shape."""
    lines = source.splitlines()
    return {
        name: sum(line == marker for line in lines) for name, marker in _MARKERS.items()
    }


def classify_params_source(source: str) -> str:
    """Return the accepted shape name, or refuse an unrecognised source."""
    counts = marker_counts(source)
    for shape, expected in _SHAPES.items():
        if counts == expected:
            return shape
    found = ", ".join(f"{name}={counts[name]}" for name in _MARKERS)
    raise RosapiClockPatchError(
        "refusing to patch rosapi params.py: source is neither the known "
        "Time() constructor, the node-clock replacement, nor the upstream "
        f"monotonic correction ({found})"
    )


def apply_params_source(source: str) -> PatchResult:
    """Rewrite the known constructor, or keep an already-corrected source."""
    shape = classify_params_source(source)
    if shape != SHAPE_BUGGY:
        return PatchResult(shape, source)

    lines = source.splitlines()
    indexes = [index for index, line in enumerate(lines) if line == BUGGY_CONSTRUCTOR]
    if len(indexes) != 1:
        raise RosapiClockPatchError(
            "refusing to patch rosapi params.py: expected exactly one "
            f"Time() constructor, found {len(indexes)}"
        )
    lines[indexes[0]] = NODE_CLOCK_CONSTRUCTOR
    patched = "\n".join(lines)
    if source.endswith("\n"):
        patched += "\n"
    if classify_params_source(patched) != SHAPE_NODE_CLOCK:
        raise RosapiClockPatchError(
            "refusing to patch rosapi params.py: replacement did not produce "
            "the node-clock constructor"
        )
    return PatchResult(ACTION_PATCHED, patched)


def resolve_params_path(ros_root: Path) -> Path:
    """Find the single installed rosapi params.py under a ROS prefix."""
    matches = sorted(ros_root.glob("lib/python*/site-packages/rosapi/params.py"))
    if len(matches) != 1:
        found = ", ".join(str(match) for match in matches) or "none"
        raise RosapiClockPatchError(
            "expected exactly one rosapi params.py under "
            f"{ros_root}/lib/python*/site-packages, found {len(matches)}: {found}"
        )
    return matches[0]


def patch_params_file(path: Path) -> str:
    """Patch ``path`` in place. Return the action. Leave drifted files untouched."""
    if not path.is_file():
        raise RosapiClockPatchError(f"rosapi params.py not found: {path}")
    original = path.read_text(encoding="utf-8")
    result = apply_params_source(original)
    if result.source != original:
        path.write_text(result.source, encoding="utf-8")
    return result.action


def _params_path_from_args(params_py: Path | None, ros_root: Path | None) -> Path:
    if params_py is not None and ros_root is not None:
        raise RosapiClockPatchError("pass a params.py path or --ros-root, not both")
    if params_py is not None:
        return params_py
    if ros_root is None:
        distro = os.environ.get("ROS_DISTRO")
        if not distro:
            raise RosapiClockPatchError(
                "pass a params.py path or --ros-root (ROS_DISTRO is not set)"
            )
        ros_root = Path("/opt/ros") / distro
    return resolve_params_path(ros_root)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params_py", nargs="?", type=Path)
    parser.add_argument("--ros-root", type=Path)
    args = parser.parse_args(argv)
    try:
        path = _params_path_from_args(args.params_py, args.ros_root)
        action = patch_params_file(path)
    except RosapiClockPatchError as exc:
        print(f"rosapi parameter-cache clock patch failed: {exc}", file=sys.stderr)
        return 1
    print(f"rosapi parameter-cache clock: {action} ({path})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
