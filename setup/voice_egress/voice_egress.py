#!/usr/bin/env python3
"""Generate the voice-container egress policy from the versioned allowlist.

The allowlist is the only list of destinations. The nftables ruleset is
rendered from it at apply time, together with runtime facts (veth, bridge
subnet, resolved addresses). A second hand-written ruleset would drift.

The policy limits destinations (address and port), not URL paths on a host.
Docker embedded DNS at 127.0.0.11 is inside the container network namespace
and is not filtered here. IPv6 egress from the voice container is rejected.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
import stat
import sys
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Sequence

TABLE = "pib_voice_egress"
PATH_LIMITATION = (
    "The policy limits destinations (address and port), not URL paths on a host."
)
DNS_NOTE = (
    "Docker embedded DNS at 127.0.0.11 is inside the container network namespace "
    "and is not filtered here. A name can still resolve; the connection to the "
    "resolved address is what this policy accepts or rejects."
)
IPV6_NOTE = (
    "IPv6 egress from the voice container is rejected so an AAAA answer is not "
    "an alternate route."
)
ALWAYS_MODES = frozenset({"common"})
REQUIRED_IDS = ("pib-cloud", "ollama", "pib-network", "gemini")
CLOUD_ADDRESS = "192.168.1.252"
CLOUD_PORT = 8090
GEMINI_MODES = frozenset({"direct", "smart", "live"})
SECRET_BASENAMES = frozenset({".env", "credentials.json", "auth.json", "secrets.json"})
PROBE_TIMEOUT_SECONDS = 3
ROLLBACK_DEADLINE_SECONDS = 20
_IFACE_CHARS = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")


class PolicyError(Exception):
    """The allowlist or the runtime facts cannot produce a safe ruleset."""


@dataclass(frozen=True)
class Entry:
    id: str
    enabled: bool
    modes: tuple[str, ...]
    destination: str
    port: Optional[int]
    protocol: str
    resolve: str
    hook: str
    purpose: str
    evidence: str


@dataclass(frozen=True)
class Allowlist:
    version: int
    table: str
    container_name: str
    entries: tuple[Entry, ...]


@dataclass(frozen=True)
class RuntimeBinding:
    veth: str
    bridge: str
    bridge_subnet: str
    gateway: str
    dns_addresses: Mapping[str, tuple[str, ...]]


def constants() -> dict[str, int]:
    return {
        "PROBE_TIMEOUT": PROBE_TIMEOUT_SECONDS,
        "ROLLBACK_DEADLINE": ROLLBACK_DEADLINE_SECONDS,
    }


def load_allowlist(path: str) -> Allowlist:
    with open(path, encoding="utf-8") as handle:
        raw = json.load(handle)
    if not isinstance(raw, dict):
        raise PolicyError("allowlist root must be an object")
    return allowlist_from_dict(raw)


def allowlist_from_dict(raw: Mapping[str, object]) -> Allowlist:
    version = raw.get("version")
    if version != 1:
        raise PolicyError("allowlist version must be 1")
    table = raw.get("table")
    if table != TABLE:
        raise PolicyError(f"allowlist table must be {TABLE}")
    subject = raw.get("subject")
    if not isinstance(subject, dict):
        raise PolicyError("allowlist subject must be an object")
    container = subject.get("container_name")
    if not isinstance(container, str) or not container:
        raise PolicyError("allowlist subject.container_name is required")
    raw_entries = raw.get("entries")
    if not isinstance(raw_entries, list) or not raw_entries:
        raise PolicyError("allowlist entries must be a non-empty list")
    entries = tuple(_entry_from_dict(item) for item in raw_entries)
    ids = [entry.id for entry in entries]
    if len(ids) != len(set(ids)):
        raise PolicyError("allowlist entry ids must be unique")
    parsed = Allowlist(
        version=1,
        table=TABLE,
        container_name=container,
        entries=entries,
    )
    preflight_static(parsed)
    return parsed


def _entry_from_dict(item: object) -> Entry:
    if not isinstance(item, dict):
        raise PolicyError("allowlist entry must be an object")
    entry_id = item.get("id")
    if (
        not isinstance(entry_id, str)
        or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,31}", entry_id) is None
    ):
        raise PolicyError("allowlist entry id is required")
    enabled = item.get("enabled")
    if not isinstance(enabled, bool):
        raise PolicyError(f"{entry_id}: enabled must be a boolean")
    modes = item.get("modes")
    if (
        not isinstance(modes, list)
        or not modes
        or not all(isinstance(mode, str) and mode for mode in modes)
    ):
        raise PolicyError(f"{entry_id}: modes must be a non-empty list of strings")
    destination = item.get("destination")
    if not isinstance(destination, str) or not destination:
        raise PolicyError(f"{entry_id}: destination is required")
    port = item.get("port")
    if port is not None and (isinstance(port, bool) or not isinstance(port, int)):
        raise PolicyError(f"{entry_id}: port must be an integer or null")
    if isinstance(port, int) and not 1 <= port <= 65535:
        raise PolicyError(f"{entry_id}: port out of range")
    protocol = item.get("protocol")
    if protocol not in ("tcp", "any"):
        raise PolicyError(f"{entry_id}: protocol must be tcp or any")
    resolve = item.get("resolve")
    if resolve not in (
        "literal",
        "dns",
        "bridge-gateway",
        "bridge-subnet",
        "multicast",
    ):
        raise PolicyError(f"{entry_id}: unknown resolve kind")
    hook = item.get("hook")
    if hook not in ("forward", "input"):
        raise PolicyError(f"{entry_id}: hook must be forward or input")
    purpose = item.get("purpose")
    evidence = item.get("evidence")
    if not isinstance(purpose, str) or not purpose:
        raise PolicyError(f"{entry_id}: purpose is required")
    if not isinstance(evidence, str) or not evidence:
        raise PolicyError(f"{entry_id}: evidence is required")
    if protocol == "tcp" and port is None:
        raise PolicyError(f"{entry_id}: tcp entries need a port")
    if resolve == "literal":
        _parse_literal(entry_id, destination)
    if resolve == "multicast":
        network = ipaddress.ip_network(destination, strict=False)
        if not network.is_multicast:
            raise PolicyError(f"{entry_id}: multicast destination is not multicast")
    return Entry(
        id=entry_id,
        enabled=enabled,
        modes=tuple(modes),
        destination=destination,
        port=port,
        protocol=protocol,
        resolve=resolve,
        hook=hook,
        purpose=purpose,
        evidence=evidence,
    )


def _parse_literal(entry_id: str, destination: str) -> None:
    try:
        ipaddress.ip_address(destination)
    except ValueError as exc:
        raise PolicyError(f"{entry_id}: literal destination must be an IP") from exc


def preflight_static(allowlist: Allowlist) -> None:
    """Refuse a file that would drop the cloud API, the bridge, or Gemini."""
    by_id = {entry.id: entry for entry in allowlist.entries}
    for required in REQUIRED_IDS:
        entry = by_id.get(required)
        if entry is None or not entry.enabled:
            raise PolicyError(f"required allowlist entry {required} is missing")
    cloud = by_id["pib-cloud"]
    if (
        cloud.destination != CLOUD_ADDRESS
        or cloud.port != CLOUD_PORT
        or cloud.protocol != "tcp"
        or cloud.resolve != "literal"
        or cloud.hook != "forward"
    ):
        raise PolicyError(
            f"required cloud destination {CLOUD_ADDRESS}:{CLOUD_PORT}/tcp is missing"
        )
    network = by_id["pib-network"]
    if network.resolve != "bridge-subnet" or network.hook != "forward":
        raise PolicyError("pib-network peers must stay allowed on the bridge")
    ollama = by_id["ollama"]
    if ollama.port != 11434 or ollama.hook != "input" or ollama.protocol != "tcp":
        raise PolicyError("ollama must stay allowed on the bridge gateway port 11434")
    gemini = by_id["gemini"]
    if (
        gemini.resolve != "dns"
        or gemini.port != 443
        or gemini.protocol != "tcp"
        or not GEMINI_MODES <= set(gemini.modes)
    ):
        raise PolicyError(
            "gemini must stay allowed for direct, smart, and live on port 443"
        )


def active_entries(
    allowlist: Allowlist, modes: Optional[Iterable[str]] = None
) -> tuple[Entry, ...]:
    """Enabled entries. ``common`` is always kept. A mode filter narrows the rest."""
    requested = None if modes is None else frozenset(modes) | ALWAYS_MODES
    selected = []
    for entry in allowlist.entries:
        if not entry.enabled:
            continue
        if requested is not None and requested.isdisjoint(entry.modes):
            continue
        selected.append(entry)
    return tuple(selected)


def validate_selection(entries: Sequence[Entry]) -> None:
    """A mode filter must not drop a required capability."""
    by_id = {entry.id: entry for entry in entries}
    for required in REQUIRED_IDS:
        if required not in by_id:
            raise PolicyError(f"selection would remove required destination {required}")
    gemini = by_id["gemini"]
    if not GEMINI_MODES <= set(gemini.modes):
        raise PolicyError("selection would remove direct, smart, or live")


def is_secret_filename(name: str) -> bool:
    """True for credential filenames. Config, persona, and memory files are not."""
    if name in SECRET_BASENAMES:
        return True
    if name.endswith(".env") and name != "blank.env":
        return True
    return False


def secret_relpaths(root: str) -> list[str]:
    """Relative paths of secret filenames under ``root``.

    File contents are not opened. Symlinks are named, not followed.
    """
    found: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames.sort()
        for name in sorted(filenames):
            if not is_secret_filename(name):
                continue
            full = os.path.join(dirpath, name)
            try:
                mode = os.lstat(full).st_mode
            except OSError:
                continue
            if stat.S_ISREG(mode) or stat.S_ISLNK(mode):
                found.append(os.path.relpath(full, root))
    return found


def _iface(name: str, label: str) -> str:
    if (
        not name
        or len(name) > 15
        or any(character not in _IFACE_CHARS for character in name)
    ):
        raise PolicyError(f"unsafe {label} name")
    return name


def _binding_network(binding: RuntimeBinding) -> ipaddress.IPv4Network:
    try:
        network = ipaddress.ip_network(binding.bridge_subnet, strict=True)
    except ValueError as exc:
        raise PolicyError("bridge subnet must be an IPv4 network") from exc
    if not isinstance(network, ipaddress.IPv4Network):
        raise PolicyError("bridge subnet must be IPv4")
    if not 8 <= network.prefixlen <= 30:
        raise PolicyError("bridge subnet prefix is not a docker bridge")
    cloud = ipaddress.ip_address(CLOUD_ADDRESS)
    if cloud in network:
        raise PolicyError(
            "bridge subnet contains the cloud host; refusing an over-broad allow"
        )
    try:
        gateway = ipaddress.ip_address(binding.gateway)
    except ValueError as exc:
        raise PolicyError("bridge gateway must be an IPv4 address") from exc
    if not isinstance(gateway, ipaddress.IPv4Address) or gateway not in network:
        raise PolicyError("bridge gateway is not inside the bridge subnet")
    return network


def _dns_ips(entry: Entry, binding: RuntimeBinding) -> tuple[str, ...]:
    raw = binding.dns_addresses.get(entry.destination, ())
    if not raw:
        raise PolicyError(f"no addresses resolved for {entry.destination}")
    addresses = []
    for item in raw:
        try:
            parsed = ipaddress.ip_address(item)
        except ValueError as exc:
            raise PolicyError(f"bad address for {entry.destination}") from exc
        if not isinstance(parsed, ipaddress.IPv4Address):
            raise PolicyError(f"IPv6 is not an allow for {entry.destination}")
        addresses.append(str(parsed))
    return tuple(sorted(set(addresses), key=ipaddress.ip_address))


def tcp_allows(
    entries: Sequence[Entry], binding: RuntimeBinding
) -> list[tuple[str, int, str]]:
    """``(address, port, rule-id)`` for every enabled TCP destination."""
    pairs: list[tuple[str, int, str]] = []
    for entry in entries:
        if entry.protocol != "tcp" or entry.port is None:
            continue
        if entry.resolve == "literal":
            addresses = (entry.destination,)
        elif entry.resolve == "dns":
            addresses = _dns_ips(entry, binding)
        elif entry.resolve == "bridge-gateway":
            addresses = (binding.gateway,)
        else:
            raise PolicyError(f"{entry.id}: tcp entry has no address")
        for address in addresses:
            pairs.append((address, entry.port, entry.id))
    pairs.sort(key=lambda item: (ipaddress.ip_address(item[0]), item[1], item[2]))
    return pairs


def render_ruleset(
    allowlist: Allowlist,
    binding: RuntimeBinding,
    modes: Optional[Iterable[str]] = None,
) -> str:
    """Nftables ruleset. Rejects only packets that enter on the voice veth."""
    entries = active_entries(allowlist, modes)
    validate_selection(entries)
    veth = _iface(binding.veth, "veth")
    bridge = _iface(binding.bridge, "bridge")
    network = _binding_network(binding)
    pairs = tcp_allows(entries, binding)
    forward_pairs = [item for item in pairs if _hook(entries, item[2]) == "forward"]
    input_pairs = [item for item in pairs if _hook(entries, item[2]) == "input"]
    if not forward_pairs:
        raise PolicyError("forward allow set would be empty")
    if not any(item[2] == "pib-cloud" for item in forward_pairs):
        raise PolicyError(f"rendered ruleset omits {CLOUD_ADDRESS}:{CLOUD_PORT}")
    if not any(entry.id == "pib-network" for entry in entries):
        raise PolicyError("rendered ruleset omits pib-network")

    lines = [
        f"# {PATH_LIMITATION}",
        f"# {DNS_NOTE}",
        f"# {IPV6_NOTE}",
        "# Other containers are not matched: reject rules sit behind iifname of",
        "# the voice veth only. Intra-bridge traffic is an explicit accept.",
        f"table inet {TABLE} {{",
        "    set allow_v4_tcp {",
        "        type ipv4_addr . inet_service",
        "        elements = {",
    ]
    rendered_pairs = ",\n".join(
        f"            {address} . {port}" for address, port, _entry_id in forward_pairs
    )
    lines.append(rendered_pairs)
    lines.extend(
        [
            "        }",
            "    }",
            "",
            "    chain forward {",
            "        type filter hook forward priority -1; policy accept;",
            f'        iifname "{veth}" jump voice_forward',
            "    }",
            "",
            "    chain input {",
            "        type filter hook input priority -1; policy accept;",
            f'        iifname "{veth}" jump voice_input',
            "    }",
            "",
            "    chain voice_forward {",
            "        meta nfproto ipv6 reject with icmpv6 type no-route",
            "        ct state established,related accept",
            (
                f"        ip daddr {network} ip daddr != {binding.gateway} "
                'accept comment "pib-network"'
            ),
        ]
    )
    if any(entry.id == "ros-dds-multicast" for entry in entries):
        lines.append(
            f'        ip daddr 224.0.0.0/4 oifname "{bridge}" '
            'accept comment "ros-dds-multicast"'
        )
        lines.append(
            f'        ip protocol igmp oifname "{bridge}" accept comment "igmp"'
        )
    lines.extend(
        [
            "        ip daddr . tcp dport @allow_v4_tcp accept",
            "        meta l4proto tcp reject with tcp reset",
            "        reject with icmp type port-unreachable",
            "    }",
            "",
            "    chain voice_input {",
            "        meta nfproto ipv6 reject with icmpv6 type no-route",
            "        ct state established,related accept",
        ]
    )
    if not input_pairs:
        raise PolicyError("input allow would be empty")
    for address, port, entry_id in input_pairs:
        lines.append(
            f'        ip daddr {address} tcp dport {port} accept comment "{entry_id}"'
        )
    lines.extend(
        [
            "        meta l4proto tcp reject with tcp reset",
            "        reject with icmp type port-unreachable",
            "    }",
            "}",
            "",
        ]
    )
    text = "\n".join(lines)
    _assert_rendered(text, veth, forward_pairs, input_pairs)
    return text


def _hook(entries: Sequence[Entry], entry_id: str) -> str:
    for entry in entries:
        if entry.id == entry_id:
            return entry.hook
    raise PolicyError(f"missing hook for {entry_id}")


def _assert_rendered(
    text: str,
    veth: str,
    forward_pairs: Sequence[tuple[str, int, str]],
    input_pairs: Sequence[tuple[str, int, str]],
) -> None:
    if PATH_LIMITATION not in text or DNS_NOTE not in text or IPV6_NOTE not in text:
        raise PolicyError("rendered ruleset lost its boundary notes")
    if f'iifname "{veth}" jump voice_forward' not in text:
        raise PolicyError("forward chain is not scoped to the voice veth")
    if "reject" not in text.split("chain voice_forward", 1)[-1]:
        raise PolicyError("voice forward chain does not reject")
    for address, port, _entry_id in forward_pairs:
        token = f"{address} . {port}"
        if token not in text:
            raise PolicyError(f"rendered ruleset omits {token}")
    for address, port, _entry_id in input_pairs:
        if f"ip daddr {address} tcp dport {port} accept" not in text:
            raise PolicyError(f"rendered ruleset omits input {address}:{port}")
    if CLOUD_ADDRESS not in text or str(CLOUD_PORT) not in text:
        raise PolicyError("rendered ruleset omits the cloud API")


def dns_names(allowlist: Allowlist, modes: Optional[Iterable[str]] = None) -> list[str]:
    entries = active_entries(allowlist, modes)
    validate_selection(entries)
    return [entry.destination for entry in entries if entry.resolve == "dns"]


def _parse_modes(raw: Optional[str]) -> Optional[list[str]]:
    if raw is None or raw.strip() == "":
        return None
    return [part.strip() for part in raw.split(",") if part.strip()]


def _parse_dns(items: Sequence[str]) -> dict[str, tuple[str, ...]]:
    found: dict[str, tuple[str, ...]] = {}
    for item in items:
        if "=" not in item:
            raise PolicyError(f"dns binding must be host=ip[,ip]: {item}")
        host, raw_addrs = item.split("=", 1)
        addresses = tuple(part.strip() for part in raw_addrs.split(",") if part.strip())
        if not host or not addresses:
            raise PolicyError(f"dns binding must be host=ip[,ip]: {item}")
        found[host] = addresses
    return found


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--allowlist",
        default=os.path.join(os.path.dirname(__file__), "allowlist.json"),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("preflight")
    sub.add_parser("constants")
    sub.add_parser("container-name")
    dns = sub.add_parser("dns-names")
    dns.add_argument("--modes", default=None)

    render = sub.add_parser("render")
    render.add_argument("--veth", required=True)
    render.add_argument("--bridge", required=True)
    render.add_argument("--subnet", required=True)
    render.add_argument("--gateway", required=True)
    render.add_argument("--dns", action="append", default=[])
    render.add_argument("--modes", default=None)

    secrets = sub.add_parser("secret-relpaths")
    secrets.add_argument("--root", required=True)
    return parser


def main(argv: Sequence[str]) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "constants":
            for key, value in constants().items():
                print(f"{key}={value}")
            return 0
        if args.command == "secret-relpaths":
            for relative in secret_relpaths(args.root):
                print(relative)
            return 0
        allowlist = load_allowlist(args.allowlist)
        if args.command == "preflight":
            print("preflight=ok")
            print(f"container_name={allowlist.container_name}")
            return 0
        if args.command == "container-name":
            print(allowlist.container_name)
            return 0
        if args.command == "dns-names":
            for name in dns_names(allowlist, _parse_modes(args.modes)):
                print(name)
            return 0
        if args.command == "render":
            binding = RuntimeBinding(
                veth=args.veth,
                bridge=args.bridge,
                bridge_subnet=args.subnet,
                gateway=args.gateway,
                dns_addresses=_parse_dns(args.dns),
            )
            sys.stdout.write(
                render_ruleset(allowlist, binding, _parse_modes(args.modes))
            )
            return 0
    except PolicyError as exc:
        print(f"policy_error={exc}", file=sys.stderr)
        return 1
    print("policy_error=unknown command", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
