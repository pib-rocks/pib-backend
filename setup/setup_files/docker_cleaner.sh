#!/bin/bash
# filepath: c:\Prog\Repos\pib-backend\setup\setup_files\docker_cleaner.sh

CONFIG_DIR="/home/pib/app/pib-backend/setup/setup_files"
CONTAINER_NAMES_FILE="${CONFIG_DIR}/container-names.txt"
LOG_FILE="${CONFIG_DIR}/docker-container-cleanup.log"

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

REMOVED=0

# Read each container name from the file and remove stopped containers
while read -r name; do
    # Skip empty lines and comments
    [[ -z "$name" || "$name" =~ ^# ]] && continue

    # Get IDs of stopped containers matching the name
    ids=$(docker ps -a --filter "name=$name" --filter "status=exited" --format "{{.ID}}")

    # Remove each stopped container and increment counter
    for id in $ids; do
        docker rm "$id" >> "$LOG_FILE" 2>&1 && ((REMOVED++))
    done
done < "$CONTAINER_NAMES_FILE"

log "Cleanup finished. Removed $REMOVED