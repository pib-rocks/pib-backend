#!/bin/bash
#
# Install the robot model (URDF reduction + joint mapping) the 3D tab loads.
#
# A fresh installation without it answers 404 for /robot-model/model.json and the console shows
# "model.json: HTTP 404" - that happened on 2026-11-02 and was repaired by hand. The model is not
# part of the images: it is a release asset of pib-rocks/pib-3d, so setup and update have to fetch
# it.
#
# Idempotent: when the installed version.json already names the requested tag, nothing is fetched.
#
# Overrides: PIB_ROBOT_MODEL_TAG, PIB_ROBOT_MODEL_SHA256, PIB_ROBOT_MODEL_URL.

set -uo pipefail

APP_DIR="${APP_DIR:-$HOME/app}"
CONTAINER="${PIB_FLASK_CONTAINER:-pib-backend-flask-app-1}"
# Setup runs this right after the stack starts, and the update service runs it
# while a container may still be coming up. Failing on the first `docker ps`
# used to report "FAILED rc=1 0s" with no hint that the model was never
# downloaded.
CONTAINER_WAIT_SECONDS="${PIB_ROBOT_MODEL_CONTAINER_WAIT:-60}"
DEST_IN_CONTAINER="/data/robot-model"
LOG_PREFIX="${LOG_PREFIX:-robot-model}"

# The pin lives next to pib_sdk_version.txt so a model release is one file to bump.
PIN_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/robot_model_version.txt"
DEFAULT_TAG="v0.11.6"
DEFAULT_SHA256="7954575a4bca8942a074abfebd9744e0ead5e6e20bf99a310e82b50c680cc251"
if [ -f "$PIN_FILE" ]; then
    DEFAULT_TAG="$(sed -n '1p' "$PIN_FILE" | tr -d '[:space:]')"
    DEFAULT_SHA256="$(sed -n '2p' "$PIN_FILE" | tr -d '[:space:]')"
fi

TAG="${PIB_ROBOT_MODEL_TAG:-$DEFAULT_TAG}"
SHA256="${PIB_ROBOT_MODEL_SHA256:-$DEFAULT_SHA256}"
URL="${PIB_ROBOT_MODEL_URL:-https://github.com/pib-rocks/pib-3d/releases/download/${TAG}/robot-model-${TAG}.zip}"

log() { echo "[${LOG_PREFIX}] $*"; }

have() { command -v "$1" >/dev/null 2>&1; }

download() {
    if have curl; then
        curl -fsSL "$1" -o "$2"
    elif have wget; then
        wget -q "$1" -O "$2"
    else
        log "neither curl nor wget available"
        return 1
    fi
}

installed_version() {
    docker exec "$CONTAINER" cat "${DEST_IN_CONTAINER}/version.json" 2>/dev/null |
        python3 -c 'import json,sys; print(json.load(sys.stdin).get("version",""))' 2>/dev/null
}

if ! docker info >/dev/null 2>&1; then
    log "docker is not usable for $(id -un) - is the docker group active for this session?"
    exit 2
fi

wait_for_container() {
    local waited=0
    while ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; do
        if [ "$waited" -ge "$CONTAINER_WAIT_SECONDS" ]; then
            return 1
        fi
        if [ "$waited" -eq 0 ]; then
            log "waiting up to ${CONTAINER_WAIT_SECONDS}s for ${CONTAINER} to start"
        fi
        sleep 2
        waited=$((waited + 2))
    done
    return 0
}

if ! wait_for_container; then
    log "container ${CONTAINER} is not running after ${CONTAINER_WAIT_SECONDS}s - model not installed"
    exit 1
fi

current="$(installed_version)"
if [ "$current" = "$TAG" ]; then
    log "already installed: ${current}"
    exit 0
fi

archive="$(mktemp)"
log "downloading ${TAG} from ${URL}"
if ! download "$URL" "$archive"; then
    log "download failed"
    rm -f "$archive"
    exit 1
fi

echo "${SHA256}  ${archive}" | sha256sum -c - >/dev/null 2>&1 || {
    log "sha256 mismatch for ${archive} (expected ${SHA256})"
    rm -f "$archive"
    exit 1
}

docker cp "$archive" "${CONTAINER}:/tmp/robot-model.zip" || { rm -f "$archive"; exit 1; }
rm -f "$archive"

docker exec "$CONTAINER" python - "$DEST_IN_CONTAINER" <<'PY' || exit 1
import json
import pathlib
import sys
import zipfile

dest = pathlib.Path(sys.argv[1])
dest.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile("/tmp/robot-model.zip") as archive:
    archive.extractall(dest)
version = json.loads((dest / "version.json").read_text())
mapping = json.loads((dest / "mapping.json").read_text())
print(f"installed robot model {version.get('version')} ({len(mapping['joints'])} joints)")
PY

docker exec "$CONTAINER" rm -f /tmp/robot-model.zip 2>/dev/null
log "done: $(installed_version)"
