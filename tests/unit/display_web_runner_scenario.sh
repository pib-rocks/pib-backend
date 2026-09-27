#!/bin/bash
# Prove the display-web runner with a stub Chromium on PATH.
# No robot, no systemd, no real browser.

set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"

RUNNER="$ROOT/setup/display_web_runner.sh"
MODULE="$ROOT/ros_packages/display/display/display_web_request.py"
DISPLAY_ROOT="$ROOT/ros_packages/display"

echo "WORKTREE_PWD=$(pwd -P)"
echo "MODULE_PATH=$(realpath "$MODULE")"
echo "RUNNER_PATH=$(realpath "$RUNNER")"

imported=$(
    PYTHONPATH="$DISPLAY_ROOT${PYTHONPATH:+:$PYTHONPATH}" python3 -c \
        'import display.display_web_request as module, os; print(os.path.realpath(module.__file__))'
)
echo "IMPORTED_MODULE=$imported"
if [ "$imported" != "$(realpath "$MODULE")" ]; then
    echo "imported module is not the worktree copy" >&2
    exit 1
fi

WORK=$(mktemp -d)
STUB_BIN="$WORK/bin"
STUB_PIDS="$WORK/stub-pids"
STUB_ARGS="$WORK/stub-args"
mkdir -p "$STUB_BIN"
: >"$STUB_PIDS"
: >"$STUB_ARGS"

cat >"$STUB_BIN/chromium" <<'EOF'
#!/bin/bash
printf '%s\n' "$$" >>"$STUB_PIDS"
printf '%s\n' "$*" >>"$STUB_ARGS"
if [ "${STUB_FAIL:-0}" = "1" ]; then
    exit 1
fi
sleep "${STUB_SLEEP:-60}"
EOF
chmod 755 "$STUB_BIN/chromium"

kill_leftovers() {
    if [ -f "$STUB_PIDS" ]; then
        local pid
        while read -r pid; do
            [ -n "$pid" ] || continue
            kill -s TERM -- "-${pid}" 2>/dev/null || kill -s TERM -- "$pid" 2>/dev/null || true
        done <"$STUB_PIDS"
    fi
    rm -rf "$WORK"
}
trap kill_leftovers EXIT

write_request() {
    local action="$1"
    local url="${2:-}"
    DISPLAY_ROOT="$DISPLAY_ROOT" python3 -c '
import os, sys
from pathlib import Path
sys.path.insert(0, os.environ["DISPLAY_ROOT"])
from display.display_web_request import write_hide_request, write_open_request
directory = Path(sys.argv[1])
if sys.argv[2] == "open":
    write_open_request(directory, sys.argv[3])
else:
    write_hide_request(directory)
' "$UPDATE" "$action" "$url"
}

status_state() {
    DISPLAY_ROOT="$DISPLAY_ROOT" python3 -c '
import os, sys
from pathlib import Path
sys.path.insert(0, os.environ["DISPLAY_ROOT"])
from display.display_web_request import read_status
document = read_status(Path(sys.argv[1]))
print("missing" if document is None else document.get("state"))
' "$UPDATE"
}

pid_count() {
    wc -l <"$STUB_PIDS" | tr -d '[:space:]'
}

run_runner() {
    PIB_UPDATE_DIR="$UPDATE" \
        PIB_BACKEND_DIR="$ROOT" \
        PIB_DISPLAY_WEB_USER_DATA_DIR="$UPDATE/profile" \
        PATH="$STUB_BIN:$PATH" \
        STUB_PIDS="$STUB_PIDS" \
        STUB_ARGS="$STUB_ARGS" \
        STUB_FAIL="${STUB_FAIL:-0}" \
        "$RUNNER"
}

UPDATE="$WORK/update"
mkdir -p "$UPDATE"
STUB_FAIL=0

write_request open "http://localhost"
run_runner
first=$(pid_count)
if [ "$first" != "1" ]; then
    echo "expected one browser, saw $first" >&2
    exit 1
fi
if [ -e "$UPDATE/display-web.json" ]; then
    echo "request file left after a successful open" >&2
    exit 1
fi
if [ "$(status_state)" != "done" ]; then
    echo "open status was $(status_state)" >&2
    exit 1
fi
if ! grep -q -- '--kiosk' "$STUB_ARGS"; then
    echo "chromium was not started with --kiosk" >&2
    exit 1
fi
if ! grep -q -- '--user-data-dir=' "$STUB_ARGS"; then
    echo "chromium was not started with its own user data directory" >&2
    exit 1
fi
browser_pid=$(head -n 1 "$STUB_PIDS")
if ! kill -0 "$browser_pid" 2>/dev/null; then
    echo "browser exited after a successful open" >&2
    exit 1
fi
echo "FIRST_OPEN_BROWSERS=$first"

write_request open "http://localhost"
run_runner
second_total=$(pid_count)
second=$((second_total - first))
if [ "$second" != "0" ]; then
    echo "second open started $second browsers" >&2
    exit 1
fi
if [ -e "$UPDATE/display-web.json" ]; then
    echo "request file left after the idempotent open" >&2
    exit 1
fi
echo "SECOND_OPEN_BROWSERS=$second"

write_request hide
run_runner
if kill -0 "$browser_pid" 2>/dev/null; then
    echo "browser still running after hide" >&2
    exit 1
fi
if [ -e "$UPDATE/display-web.pid" ]; then
    echo "pidfile left after hide" >&2
    exit 1
fi
if [ -e "$UPDATE/display-web.json" ]; then
    echo "request file left after hide" >&2
    exit 1
fi
echo "HIDE_TERMINATED=yes"
echo "PIDFILE_AFTER_HIDE=absent"

FAIL_UPDATE="$WORK/fail-update"
mkdir -p "$FAIL_UPDATE"
UPDATE="$FAIL_UPDATE"
STUB_FAIL=1
write_request open "http://localhost"
set +e
run_runner
fail_code=$?
set -e
if [ "$fail_code" -eq 0 ]; then
    echo "failing stub was treated as success" >&2
    exit 1
fi
if [ -e "$FAIL_UPDATE/display-web.json" ]; then
    echo "failing stub left display-web.json behind" >&2
    exit 1
fi
fail_state=$(status_state)
if [ "$fail_state" != "failed" ]; then
    echo "failing stub status was $fail_state" >&2
    exit 1
fi
echo "FAILING_STUB_REQUEST=absent"
echo "FAILING_STUB_STATUS=$fail_state"
