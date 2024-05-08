#!/bin/bash
#
# This script installs pibly-server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Install Pibly-Server --""$RESET_TEXT_COLOR"

PIBLY_SERVER_DIR="$USER_HOME/pibly_server"

# copy the pibly server from the setup-folder to its target in the user home-dir
cp -r "$PIBLY_SETUP_DIR/pibly_server" "$PIBLY_SERVER_DIR"

# build the pibly-server
cd "$PIBLY_SERVER_DIR"
npm install
npm run build

# create service that starts the pibly_server during boot
sudo mv "$PIBLY_SERVER_DIR/pibly_server_boot.service" /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pibly_server_boot.service

# install the pibly_client
pip install "$PIBLY_SETUP_DIR/pibly_client"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Pibly-Server installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"