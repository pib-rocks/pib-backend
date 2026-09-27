#!/bin/bash
#
# Host-side executor for fullscreen web requests. Chromium runs on the host.
# The display container only writes display-web.json; this script writes the
# status file and removes the request, on success and on failure.

set -Eeuo pipefail

UPDATE_DIR="${PIB_UPDATE_DIR:-/home/pib/app/.update}"
BACKEND_DIR="${PIB_BACKEND_DIR:-/home/pib/app/pib-backend}"
REQUEST_FILE="$UPDATE_DIR/display-web.json"
STATUS_FILE="$UPDATE_DIR/display-web-status.json"
PID_FILE="$UPDATE_DIR/display-web.pid"
LOG_FILE="$UPDATE_DIR/display-web.log"
HELPER="$BACKEND_DIR/ros_packages/display/display/display_web_request.py"
CHROMIUM_BIN="${PIB_DISPLAY_WEB_CHROMIUM:-chromium}"
USER_DATA_DIR="${PIB_DISPLAY_WEB_USER_DATA_DIR:-$HOME/.local/share/pib-display-web/chromium}"
STARTUP_POLLS="${PIB_DISPLAY_WEB_STARTUP_POLLS:-10}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

OK=false
ACTION=unknown
URL=
REQUESTED_AT=
MESSAGE=
STATUS_WRITTEN=false
CLEAN_REQUEST=0
REQUEST_INODE=

log() {
    printf '[%s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >>"$LOG_FILE" 2>/dev/null || true
}

write_status() {
    local state="$1"
    local message="$2"
    STATE="$state" MESSAGE="$message" ACTION="$ACTION" URL="$URL" \
        REQUESTED_AT="$REQUESTED_AT" STATUS_FILE="$STATUS_FILE" \
        python3 - <<'PY'
import json
import os
import tempfile
from datetime import datetime, timezone

document = {
    "schemaVersion": 1,
    "action": os.environ.get("ACTION") or "unknown",
    "state": os.environ["STATE"],
    "message": os.environ["MESSAGE"],
    "url": os.environ.get("URL") or "",
    "requestedAt": os.environ.get("REQUESTED_AT") or "",
    "updatedAt": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
}
path = os.environ["STATUS_FILE"]
descriptor, temporary = tempfile.mkstemp(
    prefix=".display-web-status.json.", dir=os.path.dirname(path)
)
try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as output:
        json.dump(document, output, sort_keys=True)
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)
except BaseException:
    try:
        os.unlink(temporary)
    except FileNotFoundError:
        pass
    raise
PY
    STATUS_WRITTEN=true
    log "$state: $message"
}

fail() {
    MESSAGE="$1"
    write_status "failed" "$MESSAGE"
    exit 1
}

remove_request_if_unchanged() {
    local current=""
    if [ ! -f "$REQUEST_FILE" ]; then
        return 0
    fi
    current=$(stat -c '%i' "$REQUEST_FILE" 2>/dev/null || true)
    # A replaced inode is a newer request from the container. Leave it so the
    # path unit can run it. The file this run actually read is removed.
    if [ -z "$REQUEST_INODE" ] || [ "$current" = "$REQUEST_INODE" ]; then
        rm -f "$REQUEST_FILE"
    fi
}

on_exit() {
    code=$?
    if [ "$CLEAN_REQUEST" = "1" ]; then
        if [ "$STATUS_WRITTEN" != "true" ]; then
            write_status "failed" "Display web runner stopped before writing status" || true
        fi
        remove_request_if_unchanged
    fi
    exit "$code"
}
trap on_exit EXIT

browser_pid_if_running() {
    local pid="" state="" cmdline=""
    if [ ! -f "$PID_FILE" ]; then
        return 1
    fi
    pid=$(tr -cd '0-9' <"$PID_FILE" || true)
    if [ -z "$pid" ]; then
        return 1
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
        return 1
    fi
    state=$(ps -o s= -p "$pid" 2>/dev/null || true)
    state=${state//[[:space:]]/}
    if [ -z "$state" ] || [ "$state" = "Z" ]; then
        return 1
    fi
    if [ ! -r "/proc/$pid/cmdline" ]; then
        return 1
    fi
    cmdline=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)
    case "$cmdline" in
        *"--user-data-dir=${USER_DATA_DIR}"*) ;;
        *) return 1 ;;
    esac
    printf '%s\n' "$pid"
}

terminate_browser() {
    local pid="$1"
    local attempt=0 state=""
    kill -s TERM -- "-${pid}" 2>/dev/null || kill -s TERM -- "$pid" 2>/dev/null || true
    while [ "$attempt" -lt 20 ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
        state=$(ps -o s= -p "$pid" 2>/dev/null || true)
        state=${state//[[:space:]]/}
        if [ -z "$state" ] || [ "$state" = "Z" ]; then
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 0.1
    done
    kill -s KILL -- "-${pid}" 2>/dev/null || kill -s KILL -- "$pid" 2>/dev/null || true
    sleep 0.1
    if ! kill -0 "$pid" 2>/dev/null; then
        return 0
    fi
    state=$(ps -o s= -p "$pid" 2>/dev/null || true)
    state=${state//[[:space:]]/}
    if [ -z "$state" ] || [ "$state" = "Z" ]; then
        return 0
    fi
    return 1
}

start_browser() {
    local url="$1"
    local pid="" attempt=0 state="" code=0
    if ! command -v "$CHROMIUM_BIN" >/dev/null 2>&1; then
        fail "Chromium is not available on PATH ($CHROMIUM_BIN)"
    fi
    mkdir -p "$USER_DATA_DIR"
    # setsid(2) in the child, then exec, so $! is the browser itself and the
    # leader of its process group. The setsid(1) helper forks when this shell
    # is already a group leader, which would record the wrong pid.
    # Own user-data-dir so this browser does not take the profile lock of a
    # manually opened Chromium. --kiosk is what makes it fullscreen: labwc has
    # no window rules. An unreachable page is Chromium's own error screen.
    python3 -c 'import errno, os, sys
try:
    os.setsid()
except OSError as exc:
    if exc.errno != errno.EPERM:
        raise
os.execvp(sys.argv[1], sys.argv[1:])' \
        "$CHROMIUM_BIN" \
        --kiosk \
        --no-first-run \
        --ozone-platform=wayland \
        --user-data-dir="$USER_DATA_DIR" \
        -- "$url" >>"$LOG_FILE" 2>&1 &
    pid=$!
    while [ "$attempt" -lt "$STARTUP_POLLS" ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            code=0
            wait "$pid" || code=$?
            fail "Chromium exited immediately with status ${code}"
        fi
        state=$(ps -o s= -p "$pid" 2>/dev/null || true)
        state=${state//[[:space:]]/}
        if [ "$state" = "Z" ]; then
            code=0
            wait "$pid" || code=$?
            fail "Chromium exited immediately with status ${code}"
        fi
        attempt=$((attempt + 1))
        sleep 0.05
    done
    printf '%s\n' "$pid" >"$PID_FILE"
    log "Chromium started pid=$pid"
}

perform_open() {
    local existing=""
    if existing=$(browser_pid_if_running); then
        log "Browser already running pid=$existing; not starting another"
        write_status "done" "Display web browser is already running"
        exit 0
    fi
    rm -f "$PID_FILE"
    start_browser "$URL"
    write_status "done" "Display web browser started"
    exit 0
}

perform_hide() {
    local existing=""
    if ! existing=$(browser_pid_if_running); then
        rm -f "$PID_FILE"
        write_status "done" "No display web browser is running"
        exit 0
    fi
    if ! terminate_browser "$existing"; then
        fail "Could not terminate display web browser ${existing}"
    fi
    rm -f "$PID_FILE"
    write_status "done" "Display web browser terminated"
    exit 0
}

if [ ! -d "$UPDATE_DIR" ] || [ ! -w "$UPDATE_DIR" ]; then
    printf 'Display web directory is not writable: %s\n' "$UPDATE_DIR" >&2
    exit 1
fi

touch "$LOG_FILE" 2>/dev/null || true

if [ ! -f "$REQUEST_FILE" ]; then
    log "No display web request is queued"
    exit 0
fi

CLEAN_REQUEST=1
REQUEST_INODE=$(stat -c '%i' "$REQUEST_FILE" 2>/dev/null || true)

if [ ! -f "$HELPER" ]; then
    fail "Display web request helper is missing: $HELPER"
fi

parsed=$(python3 "$HELPER" validate "$REQUEST_FILE" 2>>"$LOG_FILE") || fail "Could not validate display-web.json"
# The helper prints only shell assignments it quoted itself.
eval "$parsed"
if [ "$OK" != "true" ]; then
    fail "${MESSAGE:-display-web.json is invalid}"
fi

case "$ACTION" in
    open) perform_open ;;
    hide) perform_hide ;;
    *) fail "invalid request field: action" ;;
esac
