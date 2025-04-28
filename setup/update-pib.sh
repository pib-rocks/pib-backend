#!/bin/bash
set -e  # Stop on errors

# Configuration
USER="pib"
LOG_FILE="/home/$USER/update.log"

# Check correct user
if [ "$(whoami)" != "$USER" ]; then
    echo "Run this as user: $USER"
    exit 1
fi

# Setup sudo without password (temporary)
echo "Setting up temporary sudo access..."
sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/$USER"
sudo chmod 0440 "/etc/sudoers.d/$USER"

# Start logging
echo "Logging to $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
echo -e "\nUpdate started: $(date)"

# Update backend
echo -e "\nUpdating backend:"
cd "$BACKEND_DIR"
git pull
sudo docker compose --profile all up --force-recreate --build -d

# Update frontend
echo -e "\nUpdating frontend:"
cd "$FRONTEND_DIR"
git pull
sudo docker compose up --force-recreate --build -d

# Cleanup
echo -e "\nCleaning up:"
sudo rm -v "/etc/sudoers.d/$USER"

echo -e "\nUpdate successful."