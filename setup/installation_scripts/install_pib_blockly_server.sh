#!/bin/bash
#
# This script installs pib-blockly-server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Install Pib-Blockly-Server --""$RESET_TEXT_COLOR"

PIB_BLOCKLY_SERVER_DIR="$USER_HOME/pib_blockly_server"

# copy the pib-blockly-server from the setup-folder to its target in the user home-dir
cp -r "$PIB_BLOCKLY_SETUP_DIR/pib_blockly_server" "$PIB_BLOCKLY_SERVER_DIR"

# build the pib-blockly-server
cd "$PIB_BLOCKLY_SERVER_DIR"
npm install
npm run build

# create service that starts the pib_blockly_server during boot
sudo mv "$PIB_BLOCKLY_SERVER_DIR/pib_blockly_server_boot.service" /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pib_blockly_server_boot.service

# install the pib_blockly_client
pip install "$PIB_BLOCKLY_SETUP_DIR/pib_blockly_client"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Pib-Blockly-Server installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"