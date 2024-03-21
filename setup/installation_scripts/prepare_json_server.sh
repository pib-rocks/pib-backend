#!/bin/bash
#
# This script prepares everything for the JSON-Server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Prepares JSON-Server --""$RESET_TEXT_COLOR"

PIB_FOLDER=$(find /var/tmp -type d -name 'pib-temp.*' -print -quit)

echo "Folder with the folowing name found: " + $PIB_FOLDER

mkdir json-server

cp -r $PIB_FOLDER/frontend/server $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package.json $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package-lock.json $ROS_WORKING_DIR/json-server

# nvm is already installed unter /etc/nvm 
sed -i 's/export NVM_DIR="$HOME\/.nvm/export NVM_DIR="\/etc\/nvm/g' ~/.bashrc
source ~/.bashrc