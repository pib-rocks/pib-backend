#!/bin/bash
#
# This script assumes:
#   - that setup-pib was already executed
#   - the default-DEFAULT_USER "pib" is executing it

set -e  # Stop on errors

# Configuration
DEFAULT_USER="pib"
LOG_FILE="$HOME/update-pib.log"

# Check correct user
if [ "$(whoami)" != "$DEFAULT_USER" ]; then
    echo "Run this as user: $DEFAULT_USER"
    exit 1
fi

# Setup sudo without password (temporary)
echo "Setting up temporary sudo access..."
sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$DEFAULT_USER"
sudo chmod 0440 "/etc/sudoers.d/$DEFAULT_USER"

# Start logging
echo "Logging to $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
echo -e "\nUpdate started: $(date)"


function update_backend() {
    echo -e "\nUpdating backend:"
    cd "$BACKEND_DIR"
    git pull
    sudo docker compose --profile all up --force-recreate --build -d
}

function update_frontend() {
    echo -e "\nUpdating frontend:"
    cd "$FRONTEND_DIR"
    git pull
    sudo docker compose up --force-recreate --build -d
}

# Update backend
update_backend

# Update frontend
update_frontend


# Cleanup
echo -e "\nCleaning up:"
sudo rm -v "/etc/sudoers.d/$DEFAULT_USER"

echo -e "\nUpdate successful."