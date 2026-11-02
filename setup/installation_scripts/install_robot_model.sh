#!/bin/bash
#
# Install the robot model (URDF reduction + joint mapping) the 3D tab loads.
#
# The model is not part of the images: it is a release asset of pib-rocks/pib-3d, so setup and
# update have to fetch it - a fresh installation without it answers 404 for
# /robot-model/model.json and the console shows "model.json: HTTP 404".
#
# It lands in the directory the flask container mounts as /data/robot-model
# ($APP_DIR/data/robot-model). When that directory is writable it is unpacked there directly;
# otherwise (it is root-owned on an installed robot) it is unpacked through the container, which
# writes through the same mount. Either way it survives a container recreate - what did NOT work
# was unpacking into the container's own filesystem, because `docker compose up` wipes that.
#
# Idempotent: when the installed version.json already names the requested tag, nothing is fetched.
# The archive is unpacked next to the target and then swapped in, so a failed download never leaves
# a half-installed model behind.
#
# Overrides: PIB_ROBOT_MODEL_TAG, PIB_ROBOT_MODEL_SHA256, PIB_ROBOT_MODEL_URL,
#            PIB_ROBOT_MODEL_DIR (host directory), PIB_FLASK_CONTAINER.

set -uo pipefail

APP_DIR="${APP_DIR:-$HOME/app}"
CONTAINER="${PIB_FLASK_CONTAINER:-pib-backend-flask-app-1}"
DEST="${PIB_ROBOT_MODEL_DIR:-$APP_DIR/data/robot-model}"
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

container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"
}

version_of() {
    python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("version",""))' "$1" 2>/dev/null
}

installed_version() {
    if [ -f "$DEST/version.json" ]; then
        version_of "$DEST/version.json"
        return 0
    fi
    # An older installation may sit inside the container's own filesystem; read it if the mount
    # copy is missing.
    docker exec "$CONTAINER" cat "${DEST_IN_CONTAINER}/version.json" 2>/dev/null |
        python3 -c 'import json,sys; print(json.load(sys.stdin).get("version",""))' 2>/dev/null
}

download() {
    if have curl; then
        curl -fsSL --retry 2 --connect-timeout 20 "$1" -o "$2"
    elif have wget; then
        wget -q --tries=3 --timeout=20 "$1" -O "$2"
    else
        log "neither curl nor wget available"
        return 1
    fi
}

UNPACK_PY=$(cat <<'PY'
import json
import pathlib
import shutil
import sys
import zipfile

archive = pathlib.Path(sys.argv[1])
dest = pathlib.Path(sys.argv[2])
staging = dest.parent / f"{dest.name}.new"

shutil.rmtree(staging, ignore_errors=True)
staging.mkdir(parents=True)
with zipfile.ZipFile(archive) as payload:
    payload.extractall(staging)

# Archives from the pib-3d release job carry a single top-level folder; flatten it.
if not (staging / "version.json").is_file():
    folders = [p for p in staging.iterdir() if p.is_dir()]
    if len(folders) == 1 and (folders[0] / "version.json").is_file():
        for item in list(folders[0].iterdir()):
            shutil.move(str(item), str(staging / item.name))
        folders[0].rmdir()

if not (staging / "model.json").is_file():
    raise SystemExit("model.json missing in the archive")
version = json.loads((staging / "version.json").read_text())
mapping = json.loads((staging / "mapping.json").read_text())

shutil.rmtree(dest, ignore_errors=True)
staging.rename(dest)
print(f"installed robot model {version.get('version')} ({len(mapping['joints'])} joints)")
PY
)

if ! have python3; then
    log "python3 is required to unpack the model"
    exit 1
fi

current="$(installed_version)"
if [ "$current" = "$TAG" ]; then
    log "already installed: ${current}"
    exit 0
fi

workdir="$(mktemp -d)" || { log "cannot create a temporary directory"; exit 1; }
trap 'rm -rf "$workdir"' EXIT
archive="$workdir/robot-model.zip"

log "downloading ${TAG} from ${URL}"
if ! download "$URL" "$archive"; then
    log "download failed: ${URL}"
    exit 1
fi

if ! echo "${SHA256}  ${archive}" | sha256sum -c - >/dev/null 2>&1; then
    log "sha256 mismatch for ${URL} (expected ${SHA256})"
    exit 1
fi

# $APP_DIR/data is root-owned once the stack has run, so a plain login cannot create the target
# there. Unpack through the container in that case: it writes through the same bind mount.
if mkdir -p "$APP_DIR/data" 2>/dev/null && [ -w "$APP_DIR/data" ]; then
    log "unpacking into ${DEST}"
    if ! python3 -c "$UNPACK_PY" "$archive" "$DEST"; then
        log "unpacking failed - the installed model was left untouched"
        exit 1
    fi
elif container_running; then
    log "unpacking into ${DEST_IN_CONTAINER} through ${CONTAINER} (host directory is not writable)"
    docker cp "$archive" "${CONTAINER}:/tmp/robot-model.zip" || { log "cannot copy the archive into ${CONTAINER}"; exit 1; }
    if ! echo "$UNPACK_PY" | docker exec -i "$CONTAINER" python - /tmp/robot-model.zip "$DEST_IN_CONTAINER"; then
        docker exec "$CONTAINER" rm -f /tmp/robot-model.zip 2>/dev/null
        log "unpacking failed - the installed model was left untouched"
        exit 1
    fi
    docker exec "$CONTAINER" rm -f /tmp/robot-model.zip 2>/dev/null
else
    log "neither ${APP_DIR}/data is writable nor is ${CONTAINER} running - start the stack or run setup as root"
    exit 1
fi

# Verify through the container when it is up: that is what the 3D tab talks to.
if container_running; then
    seen="$(docker exec "$CONTAINER" cat "${DEST_IN_CONTAINER}/version.json" 2>/dev/null |
        python3 -c 'import json,sys; print(json.load(sys.stdin).get("version",""))' 2>/dev/null)"
    if [ "$seen" != "$TAG" ]; then
        log "installed, but ${CONTAINER} does not see ${DEST_IN_CONTAINER}/version.json (is /data mounted from ${APP_DIR}/data?)"
        exit 1
    fi
    log "done: ${seen} (visible in ${CONTAINER})"
else
    log "done: ${TAG} (installed; ${CONTAINER} is not running yet)"
fi
