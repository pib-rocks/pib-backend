#!/bin/bash
#
# This script prepare everything for the JSON-Server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Prepare JSON-Server --""$RESET_TEXT_COLOR"

PIB_FOLDER=$(find /var/tmp -type d -name 'pib-temp.*' -print -quit)

echo "Folder with the folowing name found: " + $PIB_FOLDER

mkdir json-server

cp -r $PIB_FOLDER/frontend/server $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package.json $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package-lock.json $ROS_WORKING_DIR/json-server

# nvm is already installed unter /etc/nvm 
sed -i '/echo 'export NVM_DIR="/etc/nvm"' >> ~/.bashrc/c\echo 'export NVM_DIR="$USER_HOME/nvm"' >> ~/.bashrc' ~/.bashrc
source ~/.bashrc