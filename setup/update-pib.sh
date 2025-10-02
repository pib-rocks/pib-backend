#!/bin/bash
#
# This script assumes:
#   - that setup-pib was already executed
#   - the user "pib" is executing it

# Color definitions for logging
export ERROR="\e[31m"
export WARN="\e[33m"
export SUCCESS="\e[32m"
export INFO="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

set -e  # Stop on errors

# Configuration
DEFAULT_USER="pib"
APP_DIR="$HOME/app"
BACKEND_DIR="$APP_DIR/pib-backend"
FRONTEND_DIR="$APP_DIR/cerebra"
LOG_FILE="$HOME/update-pib.log"

# Function to support printing consistent log messages
function print() {
    local color=$1
    local text=$2

    # If only one argument is provided, assume it is the text
    if [ -z "$text" ]; then
        text=$color
        color="RESET_TEXT_COLOR"
    fi

    # Check if the provided color exists
    if [ -n "$color" ] && [ -z "${!color}" ]; then
        color="RESET_TEXT_COLOR"
    fi

    # Print the text in the specified color
    echo -e "${!color}[$(date -u)][[ ${text} ]]${RESET_TEXT_COLOR}"
}

function update_backend() {
    if [ -d "$BACKEND_DIR" ]; then
        print INFO "Updating backend:"
        cd "$BACKEND_DIR" || { print ERROR "Cannot get to $BACKEND_DIR"; exit 1; }
        git pull || { print ERROR "backend git pull error"; exit 1; }
        sudo docker compose --profile all up --force-recreate --build -d || { print ERROR "docker compose backend build error"; exit 1; }
    else
        print ERROR "Directory $BACKEND_DIR does not exist"
        exit 1 
    fi
}

function update_frontend() {
    if [ -d "$FRONTEND_DIR" ]; then
        print INFO "Updating frontend:"
        cd "$FRONTEND_DIR" || { print ERROR "Cannot get to $FRONTEND_DIR"; exit 1; }
        git pull || { print ERROR "frontend git pull error"; exit 1; }
        sudo docker compose up --force-recreate --build -d || { print ERROR "docker compose frontend build error"; exit 1; }
    else
        print ERROR "Directory $FRONTEND_DIR does not exist"
        exit 1 
    fi
}

function update_docker_cleaner() {
    if [ -f "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" ]; then
        sudo cp "$BACKEND_DIR/setup/setup_files/docker_cleaner.service" /etc/systemd/system/
        sudo systemctl daemon-reload
        sudo systemctl restart docker_cleaner.service
        print SUCCESS "Docker container cleanup service updated"
    else
        print ERROR "Docker cleaner service file does not exist"
    fi

    sudo usermod -aG docker pib 
}

# Check correct user
if [ "$(whoami)" != "$DEFAULT_USER" ]; then
    print INFO "Run this as user: $DEFAULT_USER"
    exit 1
fi

# Setup sudo without password (temporary)
print INFO "Setting up temporary sudo access..."
sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$DEFAULT_USER"
sudo chmod 0440 "/etc/sudoers.d/$DEFAULT_USER"

# Start logging
print INFO "Logging to $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
print INFO "Update started: "

update_backend

update_frontend

update_docker_cleaner


# Cleanup
print INFO "Cleaning up:"
sudo rm -v "/etc/sudoers.d/$DEFAULT_USER"

print SUCCESS "Update successful."