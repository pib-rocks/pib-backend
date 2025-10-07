#!/bin/bash

CONFIG_DIR="/home/pib/app/pib-backend/setup/setup_files"
CONTAINER_NAMES_FILE="${CONFIG_DIR}/docker-image-names.txt"
LOG_FILE="${CONFIG_DIR}/docker-container-cleanup.log"

mkdir -p "$CONFIG_DIR"
touch "$LOG_FILE"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    log "ERROR: Docker not found"
    exit 1
fi

# Check if the container names file exists
if [[ ! -f "$CONTAINER_NAMES_FILE" ]]; then
    log "ERROR: $CONTAINER_NAMES_FILE not found"
    exit 1
fi

log "Starting Docker container cleanup"

REMOVED=0


# Read each image name from the file and remove stopped containers by image
while IFS= read -r image || [[ -n "$image" ]]; do
    # Skip empty lines and comments
    [[ -z "$image" || "$image" =~ ^[[:space:]]*# ]] && continue

    # Trim whitespace
    image=$(echo "$image" | xargs)

    log "Processing image: $image"

    # Get IDs of stopped containers matching the image name
    ids=$(docker ps -a --filter "ancestor=$image" --filter "status=exited" --format "{{.ID}}")

    if [[ -n "$ids" ]]; then
        log "Found stopped containers for image $image: $ids"
    else
        log "No stopped containers found for image $image"
    fi

    # Remove each stopped container and increment counter
    for id in $ids; do
        if docker rm "$id" >> "$LOG_FILE" 2>&1; then
            ((REMOVED++))
            log "Successfully removed container $id (image: $image)"
        else
            log "ERROR: Failed to remove container $id (image: $image)"
        fi
    done
done < "$CONTAINER_NAMES_FILE"

log "Cleanup finished. Removed $REMOVED containers"
