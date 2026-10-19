"""Regression for the rosapi parameter-cache clock mismatch.

The installed ros-jazzy-rosapi 2.7.1 module initialises a cached parameter
client with ``Time()`` (SYSTEM_TIME). Cleanup subtracts the node clock
(ROS_TIME) and raises ``TypeError: Can't subtract times with different clock
types``. These tests use a clock stand-in that reproduces that subtraction
rule. They do not start a ROS node, and a passing result is not live proof
that rosapi_node stays up on a robot.

The stand-in is enough to show the unpatched constructor crashes, and that the
image patch — the same node clock cleanup already uses — keeps idle eviction
and active-client preservation.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = REPO_ROOT / "ros_packages" / "rosbridge" / "Dockerfile"
PATCHER_PATH = REPO_ROOT / "ros_packages" / "rosbridge" / "patch_rosapi_param_clock.py"

# Lines copied from the captured rosapi 2.7.1 params.py (constructor, cleanup,
# and both finally-path refreshes). The patcher must recognise this shape and
# change nothing except the constructor.
OLD_PARAMS = """\
def _get_client():
    _cached_clients[service_name] = _CachedClient(use_count=0, last_used_time=Time(), client=client)
    return client


def _cleanup_timer_callback() -> None:
    assert _node is not None
    now = _node.get_clock().now()
    to_remove = []
    for service_name, cached_client in _cached_clients.items():
        if cached_client.use_count == 0 and (now - cached_client.last_used_time).nanoseconds > int(
            _client_persistence_sec * 1e9
        ):
            _node.destroy_client(cached_client.client)
            to_remove.append(service_name)


def _set_param():
    _cached_clients[service_name].use_count += 1
    try:
        pass
    finally:
        _cached_clients[service_name].use_count -= 1
        _cached_clients[service_name].last_used_time = _node.get_clock().now()


def _get_param():
    _cached_clients[service_name].use_count += 1
    try:
        pass
    finally:
        _cached_clients[service_name].use_count -= 1
        _cached_clients[service_name].last_used_time = _node.get_clock().now()
"""

# Lines copied from jazzy commit 4da27bb4266c7feb2dc09e3778bec94827e07814.
UPSTREAM_PARAMS = """\
def _get_client():
    _cached_clients[service_name] = _CachedClient(
        use_count=0, last_used_time=time.monotonic(), client=client
    )
    return client


def _cleanup_timer_callback() -> None:
    assert _node is not None
    now = time.monotonic()
    to_remove = []
    for service_name, cached_client in _cached_clients.items():
        if (
            cached_client.use_count == 0
            and now - cached_client.last_used_time > _client_persistence_sec
        ):
            _node.destroy_client(cached_client.client)
            to_remove.append(service_name)


def _set_param():
    _cached_clients[service_name].use_count += 1
    try:
        pass
    finally:
        _cached_clients[service_name].use_count -= 1
        _cached_clients[service_name].last_used_time = time.monotonic()


def _get_param():
    _cached_clients[service_name].use_count += 1
    try:
        pass
    finally:
        _cached_clients[service_name].use_count -= 1
        _cached_clients[service_name].last_used_time = time.monotonic()
"""

NODE_CLOCK_CONSTRUCTOR = (
    "    _cached_clients[service_name] = "
    "_CachedClient(use_count=0, last_used_time=_node.get_clock().now(), client=client)"
)
PERSISTENCE_NS = 5_000_000_000
ROS_TIME = "ROS_TIME"
SYSTEM_TIME = "SYSTEM_TIME"


def _load_patcher():
    spec = importlib.util.spec_from_file_location(
        "patch_rosapi_param_clock", PATCHER_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PATCHER = _load_patcher()


class Time:
    """Stand-in for rclpy.time.Time's clock-type subtraction rule."""

    def __init__(self, *, nanoseconds=0, clock_type=SYSTEM_TIME):
        self.nanoseconds = nanoseconds
        self.clock_type = clock_type

    def __sub__(self, other):
        if not isinstance(other, Time):
            return NotImplemented
        if self.clock_type != other.clock_type:
            raise TypeError("Can't subtract times with different clock types")
        return Duration(self.nanoseconds - other.nanoseconds)


class Duration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return Time(nanoseconds=self.nanoseconds, clock_type=ROS_TIME)


class Node:
    def __init__(self, nanoseconds):
        self.clock = Clock(nanoseconds)
        self.destroyed = []

    def get_clock(self):
        return self.clock

    def destroy_client(self, client):
        self.destroyed.append(client)


class CachedClient:
    def __init__(self, use_count, last_used_time, client):
        self.use_count = use_count
        self.last_used_time = last_used_time
        self.client = client


def constructor_statement(source: str) -> str:
    lines = source.splitlines()
    for index, line in enumerate(lines):
        if "_cached_clients[service_name] = _CachedClient(" not in line:
            continue
        if "last_used_time" in line and line.rstrip().endswith(")"):
            return line.strip()
        block = [line]
        for follow in lines[index + 1 :]:
            block.append(follow)
            if follow.strip() == ")":
                break
        return "\n".join(block)
    raise AssertionError("cached-client constructor not found")


def load_entry(source: str, *, nanoseconds: int):
    """Execute the source constructor the way _get_client does, before any use."""
    node = Node(nanoseconds)
    client = object()
    service_name = "/missing_node/get_parameters"
    cached: dict = {}
    exec(  # noqa: S102 - the constructor under test is a one-line assignment
        constructor_statement(source),
        {
            "Time": Time,
            "_CachedClient": CachedClient,
            "_node": node,
            "_cached_clients": cached,
            "service_name": service_name,
            "client": client,
        },
    )
    return node, client, service_name, cached


def run_cleanup(node: Node, cached: dict, persistence_sec: float = 5.0):
    """The installed cleanup condition: idle and older than persistence."""
    now = node.get_clock().now()
    to_remove = []
    for service_name, cached_client in cached.items():
        if cached_client.use_count == 0 and (
            now - cached_client.last_used_time
        ).nanoseconds > int(persistence_sec * 1e9):
            node.destroy_client(cached_client.client)
            to_remove.append(service_name)
    for service_name in to_remove:
        del cached[service_name]
    return to_remove


def patched_source() -> str:
    result = PATCHER.apply_params_source(OLD_PARAMS)
    assert result.action == "patched"
    return result.source


def test_unpatched_startup_client_cleanup_mixes_clock_types():
    """A client cached before its first successful use still hits cleanup."""
    node, _client, service_name, cached = load_entry(OLD_PARAMS, nanoseconds=1_000)
    entry = cached[service_name]
    assert entry.use_count == 0
    assert entry.last_used_time.clock_type == SYSTEM_TIME
    assert node.get_clock().now().clock_type == ROS_TIME
    with pytest.raises(
        TypeError, match="Can't subtract times with different clock types"
    ):
        run_cleanup(node, cached)
    assert service_name in cached
    assert node.destroyed == []


def test_patched_unused_client_survives_cleanup_before_first_use():
    source = patched_source()
    node, _client, service_name, cached = load_entry(source, nanoseconds=1_000)
    entry = cached[service_name]
    assert entry.use_count == 0
    assert entry.last_used_time.clock_type == ROS_TIME
    assert entry.last_used_time.nanoseconds == 1_000
    assert run_cleanup(node, cached) == []
    assert service_name in cached
    assert node.destroyed == []


def test_patched_idle_client_is_evicted_only_after_persistence():
    source = patched_source()
    node, client, service_name, cached = load_entry(source, nanoseconds=0)
    node.clock.nanoseconds = PERSISTENCE_NS
    assert run_cleanup(node, cached) == []
    assert service_name in cached
    node.clock.nanoseconds = PERSISTENCE_NS + 1
    assert run_cleanup(node, cached) == [service_name]
    assert node.destroyed == [client]
    assert service_name not in cached


def test_patched_active_client_is_kept_past_persistence():
    source = patched_source()
    node, client, service_name, cached = load_entry(source, nanoseconds=0)
    cached[service_name].use_count = 1
    node.clock.nanoseconds = 60 * PERSISTENCE_NS
    assert run_cleanup(node, cached) == []
    assert node.destroyed == []
    assert service_name in cached
    cached[service_name].use_count = 0
    assert run_cleanup(node, cached) == [service_name]
    assert node.destroyed == [client]


def test_patched_refresh_uses_the_node_clock_and_restarts_the_idle_window():
    source = patched_source()
    refresh_lines = [
        line
        for line in source.splitlines()
        if line.strip()
        == "_cached_clients[service_name].last_used_time = _node.get_clock().now()"
    ]
    assert refresh_lines == [
        "        _cached_clients[service_name].last_used_time = _node.get_clock().now()",
        "        _cached_clients[service_name].last_used_time = _node.get_clock().now()",
    ]
    node, client, service_name, cached = load_entry(source, nanoseconds=0)
    node.clock.nanoseconds = 60 * PERSISTENCE_NS
    exec(  # noqa: S102 - the installed finally-path assignment, taken from the patch
        refresh_lines[0].strip(),
        {
            "_cached_clients": cached,
            "_node": node,
            "service_name": service_name,
        },
    )
    refreshed = cached[service_name].last_used_time
    assert refreshed.clock_type == ROS_TIME
    assert refreshed.nanoseconds == 60 * PERSISTENCE_NS
    assert run_cleanup(node, cached) == []
    node.clock.nanoseconds = 60 * PERSISTENCE_NS + PERSISTENCE_NS + 1
    assert run_cleanup(node, cached) == [service_name]
    assert node.destroyed == [client]


def test_patch_changes_only_the_initial_timestamp():
    patched = patched_source()
    old_lines = OLD_PARAMS.splitlines()
    new_lines = patched.splitlines()
    diffs = [pair for pair in zip(old_lines, new_lines) if pair[0] != pair[1]]
    assert len(old_lines) == len(new_lines)
    assert diffs == [
        (
            "    _cached_clients[service_name] = "
            "_CachedClient(use_count=0, last_used_time=Time(), client=client)",
            NODE_CLOCK_CONSTRUCTOR,
        )
    ]
    assert patched.count("_cached_clients[service_name].use_count += 1") == 2
    assert patched.count("_cached_clients[service_name].use_count -= 1") == 2
    assert "cached_client.use_count == 0" in patched


def test_patch_is_idempotent_on_the_node_clock_constructor():
    once = PATCHER.apply_params_source(OLD_PARAMS)
    twice = PATCHER.apply_params_source(once.source)
    assert twice.action == "already-node-clock"
    assert twice.source == once.source
    assert PATCHER.classify_params_source(twice.source) == "already-node-clock"


def test_patch_leaves_the_upstream_monotonic_correction_unchanged():
    result = PATCHER.apply_params_source(UPSTREAM_PARAMS)
    assert result.action == "already-upstream-monotonic"
    assert result.source == UPSTREAM_PARAMS
    assert (
        PATCHER.classify_params_source(UPSTREAM_PARAMS) == "already-upstream-monotonic"
    )


@pytest.mark.parametrize(
    "source",
    [
        OLD_PARAMS.replace("Time()", "Time(seconds=0)"),
        OLD_PARAMS.replace(".nanoseconds > int(", ".nanoseconds >= int("),
        OLD_PARAMS.replace(
            "        _cached_clients[service_name].last_used_time = "
            "_node.get_clock().now()\n",
            "",
            1,
        ),
        UPSTREAM_PARAMS.replace(
            "and now - cached_client.last_used_time > _client_persistence_sec",
            "and now - cached_client.last_used_time >= _client_persistence_sec",
        ),
        OLD_PARAMS + "\n    now = time.monotonic()\n",
        "last_used_time=Time()\n",
        "",
    ],
)
def test_patch_refuses_upstream_drift_without_rewriting(source):
    with pytest.raises(PATCHER.RosapiClockPatchError, match="refusing to patch"):
        PATCHER.apply_params_source(source)


def test_cli_patches_then_accepts_the_same_file(tmp_path, capsys):
    target = tmp_path / "params.py"
    target.write_text(OLD_PARAMS, encoding="utf-8")
    assert PATCHER.main([str(target)]) == 0
    assert NODE_CLOCK_CONSTRUCTOR in target.read_text(encoding="utf-8")
    assert "patched" in capsys.readouterr().out
    assert PATCHER.main([str(target)]) == 0
    captured = capsys.readouterr()
    assert "already-node-clock" in captured.out
    assert target.read_text(encoding="utf-8").count(NODE_CLOCK_CONSTRUCTOR) == 1


def test_cli_refuses_drift_and_leaves_the_file_untouched(tmp_path, capsys):
    drifted = OLD_PARAMS.replace("Time()", "Time(clock_type=ClockType.ROS_TIME)")
    target = tmp_path / "params.py"
    target.write_text(drifted, encoding="utf-8")
    assert PATCHER.main([str(target)]) == 1
    assert target.read_text(encoding="utf-8") == drifted
    assert "refusing to patch" in capsys.readouterr().err


def test_cli_resolves_one_installed_params_path(tmp_path, capsys):
    path = tmp_path / "lib" / "python3.12" / "site-packages" / "rosapi" / "params.py"
    path.parent.mkdir(parents=True)
    path.write_text(UPSTREAM_PARAMS, encoding="utf-8")
    assert PATCHER.main(["--ros-root", str(tmp_path)]) == 0
    assert path.read_text(encoding="utf-8") == UPSTREAM_PARAMS
    assert "already-upstream-monotonic" in capsys.readouterr().out


def test_resolver_refuses_zero_or_several_params_modules(tmp_path):
    with pytest.raises(PATCHER.RosapiClockPatchError, match="expected exactly one"):
        PATCHER.resolve_params_path(tmp_path)
    first = tmp_path / "lib" / "python3.12" / "site-packages" / "rosapi"
    second = tmp_path / "lib" / "python3.13" / "site-packages" / "rosapi"
    for directory in (first, second):
        directory.mkdir(parents=True)
        (directory / "params.py").write_text(OLD_PARAMS, encoding="utf-8")
    with pytest.raises(PATCHER.RosapiClockPatchError, match="found 2"):
        PATCHER.resolve_params_path(tmp_path)


def test_cli_requires_a_path_when_the_distro_is_unknown(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    missing = tmp_path / "missing.py"
    assert PATCHER.main([str(missing)]) == 1
    assert "not found" in capsys.readouterr().err
    assert PATCHER.main([]) == 1
    assert "ROS_DISTRO is not set" in capsys.readouterr().err


def test_dockerfile_applies_the_patch_and_keeps_the_interface_builds():
    text = DOCKERFILE.read_text(encoding="utf-8")
    apt = text.index("ros-$ROS_DISTRO-rosbridge-server")
    patch = text.index(
        '/opt/pib/patch_rosapi_param_clock.py --ros-root "/opt/ros/${ROS_DISTRO}"'
    )
    datatypes = text.index("COPY ./datatypes ros2_ws/datatypes")
    button = text.index("COPY ./button_service ros2_ws/button_service")
    build = text.index("colcon build")
    assert "COPY ./rosbridge/patch_rosapi_param_clock.py" in text
    assert apt < patch < datatypes < button < build
    patch_run = next(
        line
        for line in text.splitlines()
        if "patch_rosapi_param_clock.py --ros-root" in line
    )
    assert "||" not in patch_run
    assert PATCHER_PATH.is_file()
