#!/bin/bash
#
# This script assumes:
#   - that setup-pib was already executed
#   - the default-DEFAULT_USER "pib" is executing it

source "$(dirname "$0")/setup-pib.sh"

set -e  # Stop on errors

# Configuration
DEFAULT_USER="pib"
LOG_FILE="$HOME/update-pib.log"

# Check correct user
if [ "$(whoami)" != "$DEFAULT_USER" ]; then
    print "Run this as user: $DEFAULT_USER"
    exit 1
fi

# Setup sudo without password (temporary)
print "Setting up temporary sudo access..."
sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$DEFAULT_USER"
sudo chmod 0440 "/etc/sudoers.d/$DEFAULT_USER"

# Start logging
print "Logging to $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
print "Update started: "


function update_backend() {
    if [ -d "$BACKEND_DIR" ]; then
        print "\nUpdating backend:"
        cd "$BACKEND_DIR" || { echo -e "Cannot get to $BACKEND_DIR"; exit 1; }
        git pull || { echo -e "backend git pull error"; exit 1; }
        sudo docker compose --profile all up --force-recreate --build -d || { echo -e "docker compose backend build error"; exit 1; }
    else
        print "Directory $BACKEND_DIR does not exist"
        exit 1 
    fi
}

function update_frontend() {
    if [ -d "$FRONTEND_DIR" ]; then
        print "\nUpdating frontend:"
        cd "$FRONTEND_DIR" || { echo -e "Cannot get to $FRONTEND_DIR"; exit 1; }
        git pull || { echo -e "frontend git pull error"; exit 1; }
        sudo docker compose up --force-recreate --build -d || { echo -e "docker compose frontend build error"; exit 1; }
    else
        print "Directory $FRONTEND_DIR does not exist"
        exit 1 
    fi
}

# Update backend
update_backend

# Update frontend
update_frontend


# Cleanup
print "Cleaning up:"
sudo rm -v "/etc/sudoers.d/$DEFAULT_USER"

print "Update successful."