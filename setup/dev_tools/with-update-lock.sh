#!/bin/bash
#
# Run a manual deployment command while holding the same exclusive lock as
# setup/update_runner.sh, so checkout changes cannot race an in-flight update.

set -Eeuo pipefail

UPDATE_DIR="${PIB_UPDATE_DIR:-/home/pib/app/.update}"
RUNNER_LOCK="$UPDATE_DIR/runner.lock"
STATUS_FILE="$UPDATE_DIR/status.json"
LOG_FILE="$UPDATE_DIR/update.log"

usage() {
    printf 'Usage: %s -- <command...>\n' "$(basename "$0")" >&2
    exit 2
}

[ "${1:-}" = "--" ] || usage
shift
[ "$#" -ge 1 ] || usage

if ! command -v flock >/dev/null 2>&1; then
    printf 'The flock command is required for update locking but was not found on PATH; install util-linux\n' >&2
    exit 1
fi

mkdir -p "$UPDATE_DIR"
exec 9>"$RUNNER_LOCK"
if ! flock -n 9; then
    printf 'The update runner holds %s; wait for the job to finish or inspect %s and %s before running manual deployments\n' \
        "$RUNNER_LOCK" "$STATUS_FILE" "$LOG_FILE" >&2
    exit 1
fi

exec "$@"
