#!/bin/bash
# Apply or roll back the voice-container egress policy on the robot host.
# The ruleset is rendered from allowlist.json. This script does not contain
# a second list of destinations. See docs/runbooks/voice-egress.md.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PIB_VOICE_EGRESS_PY:-$ROOT/voice_egress.py}"
ALLOW="${PIB_VOICE_EGRESS_ALLOWLIST:-$ROOT/allowlist.json}"
BLANK="${PIB_VOICE_EGRESS_BLANK:-$ROOT/blank.env}"
STATE_DIR="${PIB_VOICE_EGRESS_STATE:-/run/pib-voice-egress}"
LIB_DIR="${PIB_VOICE_EGRESS_LIB:-/var/lib/pib-voice-egress}"
VIEW="${PIB_VOICE_HERMES_VIEW:-$LIB_DIR/view}"
SRC="${PIB_HERMES_HOME:-/home/pib/.hermes}"
COVER_DIR="${PIB_VOICE_EGRESS_COVERS:-$LIB_DIR/covers}"
APPLIED="${PIB_VOICE_EGRESS_APPLIED:-$LIB_DIR/applied.nft}"
NFT="${NFT:-nft}"
TABLE="pib_voice_egress"

usage() {
  echo "usage: apply.sh preflight|prepare-view|apply|rollback|remove-view" >&2
  exit 2
}

need_root() {
  if [[ "$(id -u)" -ne 0 ]]; then
    echo "policy_error=must run as root" >&2
    exit 1
  fi
}

py() {
  python3 "$PY" --allowlist "$ALLOW" "$@"
}

load_constants() {
  # shellcheck disable=SC2046
  eval "$(py constants)"
  if [[ -z "${PROBE_TIMEOUT:-}" || -z "${ROLLBACK_DEADLINE:-}" ]]; then
    echo "policy_error=constants missing" >&2
    exit 1
  fi
}

rollback() {
  if "$NFT" list table inet "$TABLE" >/dev/null 2>&1; then
    "$NFT" delete table inet "$TABLE"
  fi
  rm -f "$APPLIED"
  echo "policy_present=false"
}

probe_http() {
  local url="$1"
  docker exec "$CONTAINER" \
    curl -sS -m "$PROBE_TIMEOUT" -o /dev/null -w '%{http_code}' "$url" 2>/dev/null \
    || true
}

probe_tcp() {
  local host="$1"
  local port="$2"
  docker exec "$CONTAINER" python3 -c \
    'import socket,sys; socket.create_connection((sys.argv[1], int(sys.argv[2])), float(sys.argv[3])).close()' \
    "$host" "$port" "$PROBE_TIMEOUT"
}

baseline() {
  local code
  code="$(probe_http "http://192.168.1.252:8090/ready")"
  if [[ "$code" != "200" ]]; then
    echo "baseline_cloud=$code" >&2
    echo "policy_error=refusing to apply; cloud :8090 is not reachable" >&2
    return 1
  fi
  echo "baseline_cloud=200"
  probe_tcp flask-app 5000
  echo "baseline_flask=open"
  probe_tcp rosbridge-ws 9090
  echo "baseline_rosbridge=open"
}

self_check() {
  local code
  code="$(probe_http "http://192.168.1.252:8090/ready")"
  echo "self_check_cloud=$code"
  if [[ "$code" != "200" ]]; then
    return 1
  fi
  if ! probe_tcp flask-app 5000; then
    echo "self_check_flask=closed" >&2
    return 1
  fi
  echo "self_check_flask=open"
  if ! probe_tcp rosbridge-ws 9090; then
    echo "self_check_rosbridge=closed" >&2
    return 1
  fi
  echo "self_check_rosbridge=open"
  # An allow that fails open still reaches example.com. Timeout or reset is blocked.
  code="$(probe_http "https://example.com/")"
  echo "self_check_unlisted=$code"
  if [[ "$code" == "200" || "$code" == "301" || "$code" == "302" ]]; then
    return 1
  fi
  return 0
}

discover() {
  local pid iflink iface
  CONTAINER="$(py container-name)"
  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "policy_error=container not found: $CONTAINER" >&2
    return 1
  fi
  pid="$(docker inspect -f '{{.State.Pid}}' "$CONTAINER")"
  if [[ -z "$pid" || "$pid" == "0" ]]; then
    echo "policy_error=container is not running" >&2
    return 1
  fi
  iflink="$(cat "/proc/${pid}/root/sys/class/net/eth0/iflink")"
  VETH=""
  for iface in /sys/class/net/*; do
    if [[ "$(cat "$iface/ifindex")" == "$iflink" ]]; then
      VETH="$(basename "$iface")"
      break
    fi
  done
  if [[ -z "$VETH" ]]; then
    echo "policy_error=voice veth not found" >&2
    return 1
  fi
  if [[ ! -e "/sys/class/net/${VETH}/master" ]]; then
    echo "policy_error=voice veth has no bridge master" >&2
    return 1
  fi
  BRIDGE="$(basename "$(readlink -f "/sys/class/net/${VETH}/master")")"
  NETWORK="$(docker inspect -f '{{range $name, $net := .NetworkSettings.Networks}}{{$name}}{{end}}' "$CONTAINER")"
  SUBNET="$(docker network inspect -f '{{range .IPAM.Config}}{{.Subnet}}{{end}}' "$NETWORK")"
  GATEWAY="$(docker network inspect -f '{{range .IPAM.Config}}{{.Gateway}}{{end}}' "$NETWORK")"
  echo "container=$CONTAINER"
  echo "veth=$VETH"
  echo "bridge=$BRIDGE"
  echo "subnet=$SUBNET"
  echo "gateway=$GATEWAY"
}

render_rules() {
  local host addrs joined
  local -a dns_args=()
  local -a modes_args=()
  if [[ -n "${PIB_VOICE_EGRESS_MODES:-}" ]]; then
    modes_args=(--modes "$PIB_VOICE_EGRESS_MODES")
  fi
  while IFS= read -r host; do
    [[ -z "$host" ]] && continue
    addrs="$(getent ahostsv4 "$host" 2>/dev/null | awk '{print $1}' | sort -u | paste -sd, - || true)"
    if [[ -z "$addrs" ]]; then
      echo "policy_error=dns resolution failed for $host" >&2
      return 1
    fi
    joined="${addrs}"
    dns_args+=(--dns "${host}=${joined}")
  done < <(py dns-names "${modes_args[@]}")
  py render \
    --veth "$VETH" \
    --bridge "$BRIDGE" \
    --subnet "$SUBNET" \
    --gateway "$GATEWAY" \
    "${dns_args[@]}" \
    "${modes_args[@]}"
}

install_rules() {
  local rendered="$1"
  local batch
  batch="$(mktemp "$STATE_DIR/batch.XXXXXX.nft")"
  if "$NFT" list table inet "$TABLE" >/dev/null 2>&1; then
    {
      echo "delete table inet $TABLE"
      cat "$rendered"
    } >"$batch"
  else
    cp "$rendered" "$batch"
  fi
  if ! "$NFT" -c -f "$batch"; then
    echo "policy_error=nft check failed" >&2
    rm -f "$batch"
    return 1
  fi
  if ! "$NFT" -f "$batch"; then
    echo "policy_error=nft apply failed" >&2
    rm -f "$batch"
    return 1
  fi
  cp "$rendered" "$APPLIED"
  rm -f "$batch"
  echo "policy_applied=true"
}

start_watchdog() {
  local stamp="$STATE_DIR/selfcheck-ok"
  rm -f "$stamp"
  (
    sleep "$ROLLBACK_DEADLINE"
    if [[ ! -f "$stamp" ]]; then
      echo "watchdog=revert" >&2
      "$NFT" delete table inet "$TABLE" >/dev/null 2>&1 || true
    fi
  ) &
  WATCHDOG_PID=$!
  echo "$WATCHDOG_PID" >"$STATE_DIR/watchdog.pid"
}

stop_watchdog() {
  local stamp="$STATE_DIR/selfcheck-ok"
  touch "$stamp"
  if [[ -n "${WATCHDOG_PID:-}" ]]; then
    kill "$WATCHDOG_PID" >/dev/null 2>&1 || true
    wait "$WATCHDOG_PID" >/dev/null 2>&1 || true
  fi
}

apply_policy() {
  local rendered
  need_root
  load_constants
  py preflight >/dev/null
  mkdir -p "$STATE_DIR" "$LIB_DIR"
  exec 9>"$STATE_DIR/lock"
  flock 9
  discover
  rendered="$(mktemp "$STATE_DIR/render.XXXXXX.nft")"
  if ! render_rules >"$rendered"; then
    rm -f "$rendered"
    return 1
  fi
  if ! baseline; then
    rm -f "$rendered"
    return 1
  fi
  if ! install_rules "$rendered"; then
    rm -f "$rendered"
    return 1
  fi
  rm -f "$rendered"
  start_watchdog
  if ! self_check; then
    echo "self_check=failed" >&2
    stop_watchdog
    rollback
    return 1
  fi
  stop_watchdog
  echo "self_check=ok"
}

prepare_view() {
  local relative target cover
  need_root
  if [[ ! -d "$SRC" ]]; then
    echo "policy_error=hermes home missing" >&2
    return 1
  fi
  if [[ ! -f "$BLANK" ]]; then
    echo "policy_error=blank cover missing" >&2
    return 1
  fi
  mkdir -p "$VIEW" "$COVER_DIR"
  if ! mountpoint -q "$VIEW"; then
    mount --bind "$SRC" "$VIEW"
  fi
  # rshared so the container's rslave bind sees the cover mounts.
  mount --make-rshared "$VIEW"
  while IFS= read -r relative; do
    [[ -z "$relative" ]] && continue
    case "$relative" in
      /*|../*|*/../*|*/..|*..*)
        echo "policy_error=unsafe secret path" >&2
        return 1
        ;;
    esac
    target="$VIEW/$relative"
    if [[ ! -e "$target" ]]; then
      echo "policy_error=secret path missing in view" >&2
      return 1
    fi
    cover="$COVER_DIR/${relative//\//__}"
    cp "$BLANK" "$cover"
    chmod 644 "$cover"
    if awk '{print $5}' /proc/self/mountinfo | grep -Fxq "$target"; then
      continue
    fi
    mount --bind "$cover" "$target"
  done < <(python3 "$PY" secret-relpaths --root "$SRC")
  echo "hermes_view=$VIEW"
  echo "prepare_view=ok"
}

remove_view() {
  local container mounts target
  need_root
  container="$(py container-name)"
  if docker inspect "$container" >/dev/null 2>&1; then
    running="$(docker inspect -f '{{.State.Running}}' "$container")"
    mounts="$(docker inspect -f '{{range .Mounts}}{{.Source}} {{.Destination}}{{println}}{{end}}' "$container")"
    if [[ "$running" == "true" ]] && grep -F "$VIEW /home/pib/.hermes" <<<"$mounts" >/dev/null; then
      echo "policy_error=voice container is using the filtered view" >&2
      return 1
    fi
  fi
  while IFS= read -r target; do
    [[ -z "$target" ]] && continue
    umount "$target"
  done < <(awk '{print $5}' /proc/self/mountinfo | awk -v view="$VIEW/" 'index($0, view)==1' | sort -r)
  if mountpoint -q "$VIEW"; then
    umount "$VIEW"
  fi
  echo "prepare_view=removed"
}

command="${1:-}"
case "$command" in
  preflight)
    py preflight
    ;;
  prepare-view)
    prepare_view
    ;;
  apply)
    apply_policy
    ;;
  rollback)
    need_root
    rollback
    ;;
  remove-view)
    remove_view
    ;;
  *)
    usage
    ;;
esac
