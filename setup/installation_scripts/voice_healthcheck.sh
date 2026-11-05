#!/bin/bash
#
# Run the same functional probe the voice container's Docker healthcheck uses.
# Used after fresh setup and after an update to catch misconfiguration (e.g. wrong API key route).
#
# Overrides: PIB_VOICE_CONTAINER, PIB_VOICE_HEALTH_ATTEMPTS, PIB_VOICE_HEALTH_INTERVAL_SECONDS

set -uo pipefail

CONTAINER="${PIB_VOICE_CONTAINER:-pib-backend-ros-voice-assistant-1}"
ATTEMPTS="${PIB_VOICE_HEALTH_ATTEMPTS:-8}"
INTERVAL="${PIB_VOICE_HEALTH_INTERVAL_SECONDS:-15}"
LOG_PREFIX="${LOG_PREFIX:-voice-health}"

PROBE_CMD='source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && python3 -m voice_assistant.voice_healthcheck'

log() { echo "[${LOG_PREFIX}] $*"; }

voice_container_exists() {
    docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"
}

if ! voice_container_exists; then
    log "no voice container ($CONTAINER); skipping"
    exit 0
fi

attempt=1
last_output=""
while [ "$attempt" -le "$ATTEMPTS" ]; do
    if last_output="$(docker exec "$CONTAINER" bash -lc "$PROBE_CMD" 2>&1)"; then
        log "ok"
        exit 0
    fi
    if [ "$attempt" -eq "$ATTEMPTS" ]; then
        reason_line="$(printf '%s\n' "$last_output" | grep 'reason=' | tail -1 || true)"
        if [ -n "$reason_line" ]; then
            printf '%s\n' "$reason_line"
        else
            # No reason= line means the exec itself failed (container stopped, docker error).
            printf '%s\n' "$last_output" | tail -5
        fi
        log "probe failed after ${ATTEMPTS} attempt(s)"
        exit 1
    fi
    log "attempt ${attempt}/${ATTEMPTS} failed, retrying in ${INTERVAL}s"
    sleep "$INTERVAL"
    attempt=$((attempt + 1))
done
