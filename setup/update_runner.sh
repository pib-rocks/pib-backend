#!/bin/bash
#
# Host-side executor for backend update requests.  This must never run in the
# flask-app container: rebuilding the stack recreates that container.

set -Eeuo pipefail

UPDATE_DIR="${PIB_UPDATE_DIR:-/home/pib/app/.update}"
BACKEND_DIR="${PIB_BACKEND_DIR:-/home/pib/app/pib-backend}"
CEREBRA_DIR="${PIB_CEREBRA_DIR:-/home/pib/app/cerebra}"
REQUEST_FILE="$UPDATE_DIR/request.json"
STATUS_FILE="$UPDATE_DIR/status.json"
CANCEL_FILE="$UPDATE_DIR/cancel.json"
CHECK_FILE="$UPDATE_DIR/check.json"
AVAILABLE_FILE="$UPDATE_DIR/available.json"
CHECK_HELPER="$BACKEND_DIR/setup/update_check.py"
if [ "${1:-}" = "--check" ]; then
    LOG_FILE="$UPDATE_DIR/check.log"
else
    LOG_FILE="$UPDATE_DIR/update.log"
fi
MIN_FREE_KIB="${PIB_UPDATE_MIN_FREE_KIB:-8388608}"
VERIFY_ATTEMPTS="${PIB_UPDATE_VERIFY_ATTEMPTS:-30}"
VERIFY_INTERVAL_SECONDS="${PIB_UPDATE_VERIFY_INTERVAL_SECONDS:-5}"
MAX_ATTEMPTS="${PIB_UPDATE_MAX_ATTEMPTS:-3}"
RETRY_DELAY_SECONDS="${PIB_UPDATE_RETRY_DELAY_SECONDS:-300}"
case "$MAX_ATTEMPTS" in
    '' | *[!0-9]*) MAX_ATTEMPTS=3 ;;
esac
[ "$MAX_ATTEMPTS" -ge 1 ] || MAX_ATTEMPTS=3
case "$RETRY_DELAY_SECONDS" in
    '' | *[!0-9]*) RETRY_DELAY_SECONDS=300 ;;
esac
HEALTHCHECK="$BACKEND_DIR/setup/update_healthcheck.py"
WATCHDOG_DECIDER="$BACKEND_DIR/setup/update_watchdog.py"
STATUS_READER="$BACKEND_DIR/setup/update_status_reader.py"
WATCHDOG_HELPER="/usr/local/sbin/pib-update-watchdog"
WATCHDOG_TARGET_FILE="$UPDATE_DIR/watchdog.target"
WATCHDOG_BUILD_TIMEOUT_US="${PIB_UPDATE_WATCHDOG_BUILD_TIMEOUT_US:-1800000000}"
PRUNE_BELOW_KIB="${PIB_UPDATE_PRUNE_BELOW_KIB:-15728640}"
# Develop checkouts are not tagged. Passing this marker as APP_VERSION keeps the
# image off the compose-file fallback, which would otherwise report an old release.
DEVELOP_VERSION_MARKER="develop"

JOB_ID="unknown"
CHANNEL="unknown"
FORCE="false"
REQUESTED_AT="unknown"
STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
CURRENT_STATE="preflight"
BACKEND_BEFORE="unknown"
CEREBRA_BEFORE="unknown"
BACKEND_TARGET="unknown"
CEREBRA_TARGET="unknown"
# D12 (PR-1794): only a service that RAN BEFORE the update and is gone afterwards
# fails the gate; pre-existing damage is reported instead of blocking, because the
# strict rule blocked the update that would have fixed the damage.
BACKEND_SERVICES_BEFORE=""
CEREBRA_SERVICES_BEFORE=""
REGRESSIONS=""
UNHEALTHY_SERVICES=""
ATTEMPT=1
PREDECESSOR_INTERRUPTED="false"
PREDECESSOR_STATE="unknown"
WATCHDOG_ORIGINAL_US=""
WATCHDOG_RESTORE_NEEDED="false"
# Set only when this update is about to change the model store, so a later
# rollback can put the previous blobs back. Empty means the store was not touched.
MODEL_STORE_SNAPSHOT=""

mkdir -p "$UPDATE_DIR"
touch "$LOG_FILE"
exec >>"$LOG_FILE" 2>&1

log() {
    printf '[%s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*"
}

read_remote_revision() {
    local name="$1"
    local directory="$2"
    local output
    REMOTE_TARGET="unknown"
    REMOTE_ERROR=""
    if ! output="$(git -C "$directory" ls-remote origin "$BRANCH" 2>&1)"; then
        REMOTE_ERROR="$output"
    else
        REMOTE_TARGET="$(printf '%s\n' "$output" | awk 'NR == 1 {print $1}')"
        if [ -z "$REMOTE_TARGET" ]; then
            REMOTE_TARGET="unknown"
            REMOTE_ERROR="Remote branch $BRANCH was not found"
        fi
    fi
    if [ -n "$REMOTE_ERROR" ]; then
        log "$name remote check failed: $(printf '%s' "$REMOTE_ERROR" | tr '\n' ' ')"
    fi
}

run_check() {
    trap 'rm -f "$CHECK_FILE"' EXIT
    if [ ! -f "$CHECK_FILE" ]; then
        log "No update availability request is queued"
        return 0
    fi

    local parsed
    if ! parsed="$(python3 "$CHECK_HELPER" validate-request "$CHECK_FILE" 2>&1)"; then
        log "Invalid check.json: $parsed"
        return 1
    fi
    eval "$parsed"

    if [ -f "$REQUEST_FILE" ]; then
        log "Refusing update availability check while an update job is active"
        return 0
    fi

    local backend_installed cerebra_installed
    backend_installed="$(git -C "$BACKEND_DIR" rev-parse HEAD 2>/dev/null || printf unknown)"
    cerebra_installed="$(git -C "$CEREBRA_DIR" rev-parse HEAD 2>/dev/null || printf unknown)"

    local backend_target backend_error cerebra_target cerebra_error
    read_remote_revision "pib-backend" "$BACKEND_DIR"
    backend_target="$REMOTE_TARGET"
    backend_error="$REMOTE_ERROR"
    read_remote_revision "cerebra" "$CEREBRA_DIR"
    cerebra_target="$REMOTE_TARGET"
    cerebra_error="$REMOTE_ERROR"

    python3 "$CHECK_HELPER" write "$AVAILABLE_FILE" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        "pib-backend" "$backend_installed" "$backend_target" "$backend_error" \
        "cerebra" "$cerebra_installed" "$cerebra_target" "$cerebra_error"
    log "Update availability check completed for channel $CHANNEL"
}

if [ "${1:-}" = "--check" ]; then
    run_check
    exit $?
fi

write_status() {
    local state="$1"
    local message="$2"
    STATE="$state" MESSAGE="$message" JOB_ID="$JOB_ID" CHANNEL="$CHANNEL" \
        STARTED_AT="$STARTED_AT" STATUS_FILE="$STATUS_FILE" \
        UNHEALTHY_SERVICES="${UNHEALTHY_SERVICES:-}" ATTEMPT="$ATTEMPT" \
        MAX_ATTEMPTS="$MAX_ATTEMPTS" \
        PREDECESSOR_INTERRUPTED="$PREDECESSOR_INTERRUPTED" python3 - <<'PY'
import json
import os
import tempfile
from datetime import datetime, timezone

document = {
    "schemaVersion": 1,
    "jobId": os.environ["JOB_ID"],
    "channel": os.environ["CHANNEL"],
    "state": os.environ["STATE"],
    "message": os.environ["MESSAGE"],
    "startedAt": os.environ["STARTED_AT"],
    "updatedAt": datetime.now(timezone.utc).isoformat(),
    "attempt": int(os.environ["ATTEMPT"]),
    "maxAttempts": int(os.environ["MAX_ATTEMPTS"]),
    "predecessorInterrupted": os.environ["PREDECESSOR_INTERRUPTED"] == "true",
}
# Reported, never blocking (D12): services that were already down before the update.
unhealthy = sorted(name for name in os.environ.get("UNHEALTHY_SERVICES", "").split() if name)
if unhealthy:
    document["unhealthyServices"] = unhealthy
path = os.environ["STATUS_FILE"]
descriptor, temporary = tempfile.mkstemp(prefix=".status.json.", dir=os.path.dirname(path))
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
    CURRENT_STATE="$state"
    log "$state: $message"
}

fail() {
    local message="$1"
    write_status "failed" "$message"
    rm -f "$REQUEST_FILE"
    exit 1
}

check_cancel() {
    if [ -f "$CANCEL_FILE" ]; then
        rm -f "$CANCEL_FILE"
        fail "Update cancelled by API request"
    fi
}

load_request() {
    [ -f "$REQUEST_FILE" ] || fail "request.json is missing"
    local parsed
    if ! parsed=$(REQUEST_FILE="$REQUEST_FILE" python3 - <<'PY'
import json
import os
import shlex

with open(os.environ["REQUEST_FILE"], encoding="utf-8") as source:
    request = json.load(source)
required = {
    "jobId": str,
    "requestedAt": str,
    "actor": str,
    "channel": str,
    "force": bool,
    "confirmation": str,
}
for name, expected_type in required.items():
    value = request.get(name)
    if type(value) is not expected_type or (expected_type is str and not value.strip()):
        raise ValueError(f"invalid request field: {name}")
if request["channel"] not in {"release", "develop"}:
    raise ValueError("invalid request channel")
if request["confirmation"] != "UPDATE":
    raise ValueError("invalid confirmation token")
print("JOB_ID=" + shlex.quote(request["jobId"]))
print("REQUESTED_AT=" + shlex.quote(request["requestedAt"]))
print("CHANNEL=" + shlex.quote(request["channel"]))
print("FORCE=" + ("true" if request["force"] else "false"))
PY
    ); then
        fail "request.json is invalid"
    fi
    eval "$parsed"
}

load_predecessor_status() {
    local parsed
    if ! parsed="$(python3 "$STATUS_READER" "$STATUS_FILE" "$JOB_ID" 2>&1)"; then
        log "WARNING: could not inspect predecessor status; starting attempt 1: $parsed"
        return 0
    fi
    eval "$parsed"
    if [ "$PREDECESSOR_INTERRUPTED" = "true" ]; then
        log "WARNING: interrupted predecessor detected in state $PREDECESSOR_STATE; continuing job $JOB_ID as attempt $ATTEMPT"
    fi
}

enforce_retry_policy() {
    local exceeded delay_remaining
    if [ "$PREDECESSOR_INTERRUPTED" != "true" ]; then
        return 0
    fi
    if ! exceeded="$(python3 "$STATUS_READER" exceeded_attempt_limit "$STATUS_FILE" "$JOB_ID" "$MAX_ATTEMPTS" 2>&1)"; then
        log "WARNING: could not evaluate attempt limit; continuing: $exceeded"
        exceeded="false"
    fi
    if [ "$exceeded" = "true" ]; then
        write_status "failed" "Update retry limit of ${MAX_ATTEMPTS} attempts exceeded for job $JOB_ID; not starting another build. See $LOG_FILE"
        rm -f "$REQUEST_FILE" "$CANCEL_FILE"
        exit 1
    fi
    # The first attempt of a job never waits, even if leftover status looks interrupted.
    [ "$ATTEMPT" -gt 1 ] || return 0
    if ! delay_remaining="$(python3 "$STATUS_READER" retry_delay_remaining "$STATUS_FILE" "$RETRY_DELAY_SECONDS" 2>&1)"; then
        log "WARNING: could not evaluate retry delay; continuing without wait: $delay_remaining"
        return 0
    fi
    case "$delay_remaining" in
        '' | *[!0-9]*)
            log "WARNING: retry delay reader returned '$delay_remaining'; continuing without wait"
            return 0
            ;;
    esac
    if [ "$delay_remaining" -gt 0 ]; then
        log "Waiting ${delay_remaining}s before attempt $ATTEMPT of $MAX_ATTEMPTS (PIB_UPDATE_RETRY_DELAY_SECONDS=$RETRY_DELAY_SECONDS)"
        sleep "$delay_remaining"
    fi
}

restore_watchdog() {
    [ "$WATCHDOG_RESTORE_NEEDED" = "true" ] || return 0
    WATCHDOG_RESTORE_NEEDED="false"
    if ! printf '%s\n' "$WATCHDOG_ORIGINAL_US" > "$WATCHDOG_TARGET_FILE"; then
        log "WARNING: could not write watchdog restore target; the update result is unchanged"
        return 0
    fi
    if sudo -n "$WATCHDOG_HELPER"; then
        log "Restored systemd watchdog timeout to ${WATCHDOG_ORIGINAL_US}us"
    else
        log "WARNING: could not restore systemd watchdog timeout with $WATCHDOG_HELPER"
    fi
    rm -f "$WATCHDOG_TARGET_FILE" || true
}

extend_watchdog_for_build() {
    local current_display original_us target_us
    current_display="$(systemctl show --property=RuntimeWatchdogUSec --value 2>/dev/null || true)"
    if [ -z "$current_display" ]; then
        log "WARNING: could not read systemd watchdog timeout; continuing without build extension"
        return 0
    fi
    if ! original_us="$(python3 "$WATCHDOG_DECIDER" parse "$current_display" 2>&1)"; then
        log "WARNING: could not parse systemd watchdog timeout '$current_display': $original_us"
        return 0
    fi
    if ! target_us="$(python3 "$WATCHDOG_DECIDER" target "$current_display" "$WATCHDOG_BUILD_TIMEOUT_US" 2>&1)"; then
        log "WARNING: could not choose a systemd watchdog build timeout: $target_us"
        return 0
    fi
    if [ "$original_us" = "0" ]; then
        log "systemd watchdog is disabled; leaving it disabled during the build"
        return 0
    fi
    if [ "$target_us" = "$original_us" ]; then
        log "systemd watchdog timeout already protects the build (${original_us}us); leaving it unchanged"
        return 0
    fi
    if [ ! -x "$WATCHDOG_HELPER" ]; then
        log "WARNING: $WATCHDOG_HELPER is unavailable; continuing without watchdog extension"
        return 0
    fi
    if ! printf '%s\n' "$target_us" > "$WATCHDOG_TARGET_FILE"; then
        log "WARNING: could not write watchdog target; continuing without watchdog extension"
        return 0
    fi

    # Arm restoration before sudo: if a signal arrives after busctl applies the
    # target but before sudo returns, the EXIT trap still restores the original.
    WATCHDOG_ORIGINAL_US="$original_us"
    WATCHDOG_RESTORE_NEEDED="true"
    trap restore_watchdog EXIT
    if sudo -n "$WATCHDOG_HELPER"; then
        log "Extended systemd watchdog timeout from ${original_us}us to ${target_us}us for the update build"
    else
        log "WARNING: sudo could not extend the systemd watchdog timeout; continuing the update"
    fi
}

dirty_files() {
    git -C "$1" status --porcelain
}

validate_repository() {
    local name="$1"
    local directory="$2"
    [ -d "$directory/.git" ] || fail "$name is not a git checkout: $directory"
    local dirty
    dirty="$(dirty_files "$directory")"
    if [ -n "$dirty" ] && [ "$FORCE" != "true" ]; then
        fail "$name has local changes; retry with force=true to discard: $(printf '%s' "$dirty" | tr '\n' ';')"
    fi
    if [ -n "$dirty" ]; then
        log "force=true; $name changes will be discarded: $(printf '%s' "$dirty" | tr '\n' ';')"
    fi
}

check_update_dir() {
    [ -d "$UPDATE_DIR" ] || fail "Update directory is missing: $UPDATE_DIR"
    [ -w "$UPDATE_DIR" ] || fail "Update directory is not writable by $(id -un): $UPDATE_DIR (owner $(stat -c '%U:%G %a' "$UPDATE_DIR" 2>/dev/null || echo unknown)); fix with: sudo install -d -o pib -g pib -m 2770 $UPDATE_DIR"
}

check_watchdog_conflict() {
    # PR-1781: systemd must be the only watchdog owner. Owning the hardware
    # watchdog itself is therefore the expected state and NOT a conflict - only a
    # second owner (the watchdog daemon or any other process holding the device)
    # is. Fail on positive evidence only.
    if dpkg-query -W -f='${Status}' watchdog 2>/dev/null | grep -q 'install ok installed'; then
        fail "Competing watchdog owner detected: the watchdog package is installed"
    fi
    if systemctl list-unit-files watchdog.service --no-legend 2>/dev/null | grep -q watchdog.service; then
        fail "Competing watchdog owner detected: watchdog.service exists"
    fi
    local runtime_watchdog
    runtime_watchdog="$(systemctl show --property=RuntimeWatchdogUSec --value 2>/dev/null || true)"
    log "systemd watchdog timeout: ${runtime_watchdog:-unknown} (systemd as the single owner is expected, see PR-1781)"

    # Who else holds /dev/watchdog*? PID 1 is systemd and expected; reading the
    # fds of other users' processes may be denied, which is not evidence of a
    # conflict, so this check reports only what it can prove.
    local fd target pid holders="" inspected=0
    for fd in /proc/[0-9]*/fd/*; do
        [ -e "$fd" ] || continue
        inspected=$((inspected + 1))
        target="$(readlink "$fd" 2>/dev/null)" || continue
        case "$target" in
            /dev/watchdog | /dev/watchdog[0-9]*)
                pid="${fd#/proc/}"
                pid="${pid%%/*}"
                [ "$pid" = "1" ] && continue
                holders="$holders $pid($target)"
                ;;
        esac
    done
    if [ -n "$holders" ]; then
        fail "Competing watchdog owner detected: open handles from${holders}"
    fi
    log "watchdog handle scan: inspected $inspected file descriptors of other processes, no competing holder found"
}

backup_database() {
    local backup_name="pibdata.db.bak-$(date -u +%Y%m%dT%H%M%SZ)"
    log "Creating WAL-safe SQLite backup $backup_name inside flask-app"
    docker compose -f "$BACKEND_DIR/docker-compose.yaml" exec -T \
        -e BACKUP_NAME="$backup_name" flask-app python3 - <<'PY'
import os
import sqlite3

source = sqlite3.connect("/app/pibdata.db")
destination = sqlite3.connect("/app/" + os.environ["BACKUP_NAME"])
try:
    source.backup(destination)
    result = destination.execute("PRAGMA integrity_check").fetchone()[0]
    if result != "ok":
        raise RuntimeError(f"backup integrity check failed: {result}")
finally:
    destination.close()
    source.close()
PY
}

# Prints the APP_VERSION for the backend checkout.
# A release merge is --no-ff, so the tag published on develop is on HEAD^2
# (docs/RELEASE.md). A fast-forward has no second parent and carries the tag
# on HEAD. A develop checkout with neither uses DEVELOP_VERSION_MARKER.
resolve_app_version() {
    local directory="$1"
    local on_second_parent="" on_head="" tag=""
    on_second_parent="$(git -C "$directory" tag --points-at 'HEAD^2' 2>/dev/null || true)"
    on_head="$(git -C "$directory" tag --points-at HEAD 2>/dev/null || true)"
    tag="${on_second_parent%%$'\n'*}"
    if [ -z "$tag" ]; then
        tag="${on_head%%$'\n'*}"
    fi
    if [ -n "$tag" ]; then
        printf '%s\n' "$tag"
        return 0
    fi
    if [ "$CHANNEL" = "develop" ]; then
        printf '%s\n' "$DEVELOP_VERSION_MARKER"
        return 0
    fi
    return 1
}

# Documented injection is `docker compose build --build-arg APP_VERSION=<version>
# flask-app` (docs/RELEASE.md). -f is required because this runner does not cd
# into the checkout. APP_VERSION is also exported: the following
# `up -d --build` rebuilds from compose interpolation
# (`${APP_VERSION:-...}`) and would otherwise overwrite that image with the
# fallback. Callers leave APP_VERSION unset only for an untagged rollback.
build_backend_stack() {
    if [ -n "${APP_VERSION:-}" ]; then
        export APP_VERSION
        log "Building flask-app with --build-arg APP_VERSION=${APP_VERSION}"
        docker compose -f "$BACKEND_DIR/docker-compose.yaml" \
            build --build-arg "APP_VERSION=${APP_VERSION}" flask-app || return 1
    else
        log "WARNING: APP_VERSION is unset; backend rebuild will use the compose-file fallback"
    fi
    docker compose -f "$BACKEND_DIR/docker-compose.yaml" --profile all \
        up -d --build --force-recreate || return 1
}

fetch_repository() {
    local directory="$1"
    local branch="$2"
    # --tags fetches every tag and never deletes a local one. --prune-tags
    # mirrors the remote, so it drops withdrawn tags and can also drop a tag
    # that points outside the fetched branch. Version resolution reads the tags
    # on HEAD^2 and HEAD; a stale tag on an unrelated commit cannot change that
    # result, while a pruned tag the resolution needs would refuse a valid
    # release. --tags cannot drop a tag the version resolution needs.
    git -C "$directory" fetch --prune --tags --force origin "$branch"
    if [ "$FORCE" = "true" ]; then
        git -C "$directory" reset --hard
        git -C "$directory" clean -fd
    fi
    git -C "$directory" reset --hard "origin/$branch"
}

wait_for_flask() {
    local attempt container_id
    for attempt in $(seq 1 60); do
        container_id="$(docker compose -f "$BACKEND_DIR/docker-compose.yaml" ps -q flask-app)"
        if [ -n "$container_id" ] \
            && [ "$(docker inspect -f '{{.State.Status}}' "$container_id" 2>/dev/null || true)" = "running" ] \
            && curl --fail --silent --max-time 2 http://127.0.0.1:5000/api/version >/dev/null; then
            return 0
        fi
        sleep 5
    done
    return 1
}

verify_migration() {
    local current heads
    current="$(docker compose -f "$BACKEND_DIR/docker-compose.yaml" exec -T flask-app \
        flask --app run db current 2>/dev/null | awk 'NF {print $1}' | tail -n 1)"
    heads="$(docker compose -f "$BACKEND_DIR/docker-compose.yaml" exec -T flask-app \
        flask --app run db heads 2>/dev/null | awk 'NF {print $1}' | tail -n 1)"
    [ -n "$current" ] && [ "$current" = "$heads" ] || {
        log "Alembic mismatch: current=${current:-unknown}, head=${heads:-unknown}"
        return 1
    }
}

snapshot_running_services() {
    local compose_file="$1"
    shift
    docker compose -f "$compose_file" "$@" ps --services --status running 2>/dev/null | sort || true
}

services_defined() {
    local compose_file="$1"
    shift
    docker compose -f "$compose_file" "$@" ps --services 2>/dev/null | sort || true
}

# Compares one stack against the snapshot taken before the update. Sets REGRESSIONS and
# UNHEALTHY_SERVICES and returns non-zero when the update lost a service.
evaluate_stack_health() {
    local compose_file="$1"
    local before="$2"
    shift 2
    local expected after output health_exit=0
    expected="$(services_defined "$compose_file" "$@")"
    after="$(snapshot_running_services "$compose_file" "$@")"
    if ! output="$(HEALTH_EXPECTED="$expected" HEALTH_BEFORE="$before" HEALTH_AFTER="$after" \
        python3 "$HEALTHCHECK" 2>&1)"; then
        health_exit=1
    fi
    log "Health check for $compose_file: $(printf '%s' "$output" | tr '\n' ' ')"
    REGRESSIONS="$REGRESSIONS $(printf '%s\n' "$output" | sed -n 's/^REGRESSIONS=//p' | tr ',' ' ')"
    UNHEALTHY_SERVICES="$UNHEALTHY_SERVICES $(printf '%s\n' "$output" | sed -n 's/^UNHEALTHY=//p' | tr ',' ' ')"
    return "$health_exit"
}

# A freshly recreated stack needs time: the API answered 5.2s after a restart on
# the pib5edu, and containers can be created/restarting for a moment. Without a
# bounded wait the gate failed on a healthy robot and rolled back every update.
wait_for_api() {
    local attempt
    for attempt in $(seq 1 "$VERIFY_ATTEMPTS"); do
        if curl --fail --silent --max-time 5 http://127.0.0.1:5000/api/version >/dev/null; then
            log "The API answered on http://127.0.0.1:5000/api/version (attempt $attempt/$VERIFY_ATTEMPTS)"
            return 0
        fi
        [ $((attempt % 5)) -eq 0 ] && log "Waiting for the API (attempt $attempt/$VERIFY_ATTEMPTS)"
        sleep "$VERIFY_INTERVAL_SECONDS"
    done
    return 1
}

# Waits (bounded) until no service that ran before the update is missing any more.
wait_for_stacks() {
    local attempt
    for attempt in $(seq 1 "$VERIFY_ATTEMPTS"); do
        REGRESSIONS=""
        UNHEALTHY_SERVICES=""
        if evaluate_stack_health "$BACKEND_DIR/docker-compose.yaml" "$BACKEND_SERVICES_BEFORE" --profile all \
            && evaluate_stack_health "$CEREBRA_DIR/docker-compose.yaml" "$CEREBRA_SERVICES_BEFORE"; then
            return 0
        fi
        [ $((attempt % 5)) -eq 0 ] && log "Waiting for containers to return (attempt $attempt/$VERIFY_ATTEMPTS): $(printf '%s' "$REGRESSIONS" | tr -s ' ')"
        sleep "$VERIFY_INTERVAL_SECONDS"
    done
    return 1
}

verify_result() {
    local backend_head cerebra_head
    if ! wait_for_api; then
        log "Verification failed: the API did not answer on http://127.0.0.1:5000/api/version"
        return 1
    fi
    if ! wait_for_stacks; then
        log "Verification failed: services that ran before the update are not running: $(printf '%s' "$REGRESSIONS" | tr -s ' ')"
        return 1
    fi
    if [ -n "$(printf '%s' "$UNHEALTHY_SERVICES" | tr -d ' ')" ]; then
        log "Tolerated pre-existing unhealthy services (decision D12): $(printf '%s' "$UNHEALTHY_SERVICES" | tr -s ' ')"
    fi
    backend_head="$(git -C "$BACKEND_DIR" rev-parse HEAD)"
    if [ "$backend_head" != "$BACKEND_TARGET" ]; then
        log "Verification failed: pib-backend is at $backend_head, expected $BACKEND_TARGET"
        return 1
    fi
    cerebra_head="$(git -C "$CEREBRA_DIR" rev-parse HEAD)"
    if [ "$cerebra_head" != "$CEREBRA_TARGET" ]; then
        log "Verification failed: cerebra is at $cerebra_head, expected $CEREBRA_TARGET"
        return 1
    fi
    log "Verification passed: API answers, every container runs, revisions match"
    return 0
}

# Atomic replace, same pattern as write_status: temp file in UPDATE_DIR, fsync, rename.
write_revision() {
    local repository="$1"
    local sha="$2"
    local revision_channel="${3:-$CHANNEL}"
    REPOSITORY="$repository" SHA="$sha" CHANNEL="$revision_channel" UPDATE_DIR="$UPDATE_DIR" \
        python3 - <<'PY'
import json
import os
import tempfile
from datetime import datetime, timezone

document = {
    "gitSha": os.environ["SHA"],
    "buildTime": datetime.now(timezone.utc).isoformat(),
    "channel": os.environ["CHANNEL"],
}
path = os.path.join(os.environ["UPDATE_DIR"], os.environ["REPOSITORY"] + ".revision.json")
descriptor, temporary = tempfile.mkstemp(prefix=".revision.", dir=os.path.dirname(path))
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
}

# setup-pib.sh defaults the blob cache to ${HOME}/app/.cache/pib-models.
# This runner is started with elevated privileges, and HOME is then not
# necessarily /home/pib, so an unset PIB_MODEL_CACHE is pinned to the pib
# user's cache. A caller-supplied PIB_MODEL_CACHE is left alone. The store
# path is not overridden: setup-pib.sh already defaults it to the absolute
# /home/pib/app/pib-models (PIB_MODEL_STORE still wins when set).
model_cache_dir() {
    printf '%s\n' "${PIB_MODEL_CACHE:-/home/pib/app/.cache/pib-models}"
}

model_store_dir() {
    printf '%s\n' "${PIB_MODEL_STORE:-/home/pib/app/pib-models}"
}

refuse_model_store_path() {
    case "$1" in
        "" | "/" | "." | ".." | "/home" | "/home/pib" | "/home/pib/app" | "/tmp" | "/var" | "/var/tmp")
            return 0
            ;;
        /*)
            return 1
            ;;
    esac
    return 0
}

snapshot_model_store() {
    local store="$1"
    local destination
    if refuse_model_store_path "$store"; then
        log "refusing to snapshot model store path '${store}'"
        return 1
    fi
    destination="$(mktemp -d "$UPDATE_DIR/model-store-snapshot.XXXXXX")"
    if [ -d "$store" ] && [ -n "$(ls -A "$store")" ]; then
        cp -a "$store"/. "$destination"/
    fi
    MODEL_STORE_SNAPSHOT="$destination"
    log "Snapshotted model store ${store} before provisioning"
}

restore_model_store_snapshot() {
    local store="$1"
    if [ -z "$MODEL_STORE_SNAPSHOT" ] || [ ! -d "$MODEL_STORE_SNAPSHOT" ]; then
        return 0
    fi
    if refuse_model_store_path "$store"; then
        log "refusing to restore model store path '${store}'"
        return 1
    fi
    log "Restoring model store ${store} from the pre-provision snapshot"
    mkdir -p -- "$store"
    find "$store" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
    if [ -n "$(ls -A "$MODEL_STORE_SNAPSHOT")" ]; then
        cp -a "$MODEL_STORE_SNAPSHOT"/. "$store"/
    fi
    rm -rf "$MODEL_STORE_SNAPSHOT"
    MODEL_STORE_SNAPSHOT=""
}

discard_model_store_snapshot() {
    if [ -n "$MODEL_STORE_SNAPSHOT" ]; then
        rm -rf "$MODEL_STORE_SNAPSHOT"
        MODEL_STORE_SNAPSHOT=""
    fi
}

# Last "Models summary: placed=N already current=N failed=N" line from --models.
# Prints the failed count. Returns 1 when that OAK line is absent: a whisper
# failure has no such line, and a crash before the summary must not look like success.
oak_summary_failed_count() {
    local output="$1"
    local summary failed
    summary="$(printf '%s\n' "$output" | grep -E 'Models summary: placed=[0-9]+ already current=[0-9]+ failed=[0-9]+' | tail -n 1 || true)"
    [ -n "$summary" ] || return 1
    failed="${summary##*failed=}"
    failed="${failed%% *}"
    case "$failed" in
        '' | *[!0-9]*)
            return 1
            ;;
    esac
    printf '%s\n' "$failed"
}

# After the release checkout: compare the store with models/manifest.yaml.
# --verify-models is offline. A match does no provisioning and no fetch.
# A mismatch runs --models, which reads the cache and fetches the release asset
# only when the cache does not match either. --models also provisions whisper
# and exits 1 when that store is not writable; the OAK summary line is the
# authority for model-store drift. failed=0 keeps the update, any other OAK
# result restores the snapshot and fails. A real OAK failure is never ignored.
sync_model_store() {
    local setup cache store output status failed
    setup="$BACKEND_DIR/setup/setup-pib.sh"
    cache="$(model_cache_dir)"
    store="$(model_store_dir)"

    if [ ! -f "$setup" ]; then
        log "Model store check failed: ${setup} is missing"
        return 1
    fi
    if [ ! -f "$BACKEND_DIR/models/manifest.yaml" ]; then
        log "Model store check failed: ${BACKEND_DIR}/models/manifest.yaml is missing"
        return 1
    fi

    log "Comparing the model store with models/manifest.yaml (offline; cache ${cache})"
    if PIB_MODEL_CACHE="$cache" bash "$setup" --verify-models; then
        log "Model store matches models/manifest.yaml; skipping provisioning"
        return 0
    fi

    log "Model store differs from models/manifest.yaml; provisioning changed blobs (cache ${cache})"
    if ! snapshot_model_store "$store"; then
        log "Model store check failed: could not snapshot ${store} before provisioning"
        return 1
    fi

    status=0
    output="$(PIB_MODEL_CACHE="$cache" bash "$setup" --models 2>&1)" || status=$?
    printf '%s\n' "$output"

    if ! failed="$(oak_summary_failed_count "$output")"; then
        log "OAK model provisioning failed: no usable Models summary (setup-pib.sh --models exit ${status})"
        restore_model_store_snapshot "$store" || log "WARNING: could not restore the model store snapshot"
        return 1
    fi
    if [ "$failed" -ne 0 ]; then
        log "OAK model provisioning failed: failed=${failed} (setup-pib.sh --models exit ${status})"
        restore_model_store_snapshot "$store" || log "WARNING: could not restore the model store snapshot"
        return 1
    fi
    if [ "$status" -ne 0 ]; then
        log "WARNING: setup-pib.sh --models exited ${status} after the OAK store was provisioned (failed=0). A non-OAK step failed (whisper). This is not model-store drift."
    else
        log "OAK model store provisioned (failed=0)"
    fi
    return 0
}

rollback_once() {
    log "Verification failed; attempting one rollback build"
    git -C "$BACKEND_DIR" reset --hard "$BACKEND_BEFORE"
    git -C "$CEREBRA_DIR" reset --hard "$CEREBRA_BEFORE"
    git -C "$CEREBRA_DIR" submodule update --init --recursive
    if ! restore_model_store_snapshot "$(model_store_dir)"; then
        fail "Verification failed and the model store snapshot could not be restored"
    fi
    if APP_VERSION="$(resolve_app_version "$BACKEND_DIR")"; then
        export APP_VERSION
        log "Rollback rebuild uses APP_VERSION=$APP_VERSION"
    else
        log "WARNING: rolled-back checkout has no release tag; backend rebuild will use the compose-file fallback"
        unset APP_VERSION
    fi
    build_backend_stack
    docker compose -f "$CEREBRA_DIR/docker-compose.yaml" \
        up -d --build --force-recreate
    BACKEND_TARGET="$BACKEND_BEFORE"
    CEREBRA_TARGET="$CEREBRA_BEFORE"
    if wait_for_flask && verify_migration && verify_result; then
        write_revision "pib-backend" "$BACKEND_BEFORE" "unknown"
        write_revision "cerebra" "$CEREBRA_BEFORE" "unknown"
        write_status "rolled_back" "Verification failed; previous revisions were rebuilt successfully"
        rm -f "$REQUEST_FILE" "$CANCEL_FILE"
        exit 1
    fi
    fail "Verification failed and the single rollback attempt also failed"
}

on_unexpected_error() {
    local exit_code=$?
    trap - ERR
    write_status "failed" "Unexpected runner error (exit $exit_code) during $CURRENT_STATE"
    rm -f "$REQUEST_FILE"
    exit "$exit_code"
}
if [ "${1:-}" = "--sync-models" ]; then
    if sync_model_store; then
        exit 0
    fi
    exit 1
fi

trap on_unexpected_error ERR

load_request
load_predecessor_status
enforce_retry_policy
write_status "preflight" "Validating repositories, disk, watchdog ownership, and database backup (attempt ${ATTEMPT} of ${MAX_ATTEMPTS})"
check_cancel
check_update_dir
BACKEND_SERVICES_BEFORE="$(snapshot_running_services "$BACKEND_DIR/docker-compose.yaml" --profile all)"
CEREBRA_SERVICES_BEFORE="$(snapshot_running_services "$CEREBRA_DIR/docker-compose.yaml")"
log "Services running before the update: pib-backend=[$(printf '%s' "$BACKEND_SERVICES_BEFORE" | tr '\n' ' ')] cerebra=[$(printf '%s' "$CEREBRA_SERVICES_BEFORE" | tr '\n' ' ')]"
validate_repository "pib-backend" "$BACKEND_DIR"
validate_repository "cerebra" "$CEREBRA_DIR"
check_watchdog_conflict
FREE_KIB="$(df -Pk "$BACKEND_DIR" | awk 'NR == 2 {print $4}')"
[ "$FREE_KIB" -ge "$MIN_FREE_KIB" ] || fail "Insufficient free disk space: ${FREE_KIB} KiB"
BACKEND_BEFORE="$(git -C "$BACKEND_DIR" rev-parse HEAD)"
CEREBRA_BEFORE="$(git -C "$CEREBRA_DIR" rev-parse HEAD)"
backup_database || fail "WAL-safe database backup failed"

check_cancel
write_status "fetching" "Fetching release targets"
if [ "$CHANNEL" = "release" ]; then
    BRANCH="main"
else
    BRANCH="develop"
fi
fetch_repository "$BACKEND_DIR" "$BRANCH" || fail "Failed to fetch pib-backend"
fetch_repository "$CEREBRA_DIR" "$BRANCH" || fail "Failed to fetch cerebra"
git -C "$CEREBRA_DIR" submodule update --init --recursive || fail "Failed to update cerebra submodules"
BACKEND_TARGET="$(git -C "$BACKEND_DIR" rev-parse HEAD)"
CEREBRA_TARGET="$(git -C "$CEREBRA_DIR" rev-parse HEAD)"
if ! APP_VERSION="$(resolve_app_version "$BACKEND_DIR")"; then
    fail "Release checkout has no git tag on ${BACKEND_TARGET} (HEAD^2 or HEAD) after fetching branch ${BRANCH} with tags; refusing to build with the compose-file APP_VERSION fallback"
fi
log "Resolved APP_VERSION=${APP_VERSION} for pib-backend at ${BACKEND_TARGET}"

check_cancel
write_status "fetching" "Comparing the model store with models/manifest.yaml"
sync_model_store || fail "OAK model store does not match models/manifest.yaml and provisioning failed"

check_cancel
extend_watchdog_for_build
write_status "building" "Building and recreating backend and cerebra containers (APP_VERSION=${APP_VERSION})"
if [ "$FREE_KIB" -lt "$PRUNE_BELOW_KIB" ]; then
    log "Disk space is below prune threshold; pruning Docker build cache"
    docker builder prune -af
fi
build_backend_stack || fail "Backend container build failed"
docker compose -f "$CEREBRA_DIR/docker-compose.yaml" \
    up -d --build --force-recreate || fail "Cerebra container build failed"

check_cancel
write_status "restarting" "Waiting for the recreated flask-app"
wait_for_flask || fail "flask-app did not become ready"

check_cancel
write_status "migrating" "Verifying the startup migration reached the Alembic head"
if ! verify_migration; then
    rollback_once
fi

check_cancel
write_status "verifying" "Verifying API, containers, and checked-out revisions"
if ! verify_result; then
    rollback_once
fi

discard_model_store_snapshot
write_revision "pib-backend" "$BACKEND_TARGET"
write_revision "cerebra" "$CEREBRA_TARGET"
write_status "done" "Update completed and verified"
rm -f "$REQUEST_FILE" "$CANCEL_FILE"
