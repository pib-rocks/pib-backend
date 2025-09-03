#!/bin/bash

CONFIG_DIR="/home/pib/app/pib-backend/setup/setup_files"
CONTAINER_NAMES_FILE="${CONFIG_DIR}/container-names.txt"
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

# Read each container name from the file and remove stopped containers
while IFS= read -r name || [[ -n "$name" ]]; do
    # Skip empty lines and comments
    [[ -z "$name" || "$name" =~ ^[[:space:]]*# ]] && continue
    
    # Trim whitespace
    name=$(echo "$name" | xargs)
    
    log "Processing container: $name"
    
    # Get IDs of stopped containers matching the name
    ids=$(docker ps -a --filter "name=$name" --filter "status=exited" --format "{{.ID}}")
    
    if [[ -n "$ids" ]]; then
        log "Found stopped containers for $name: $ids"
    else
        log "No stopped containers found for $name"
    fi
    
    # Remove each stopped container and increment counter
    for id in $ids; do
        if docker rm "$id" >> "$LOG_FILE" 2>&1; then
            ((REMOVED++))
            log "Successfully removed container $id ($name)"
        else
            log "ERROR: Failed to remove container $id ($name)"
        fi
    done
done < "$CONTAINER_NAMES_FILE"

log "Cleanup finished. Removed $REMOVED containers"
