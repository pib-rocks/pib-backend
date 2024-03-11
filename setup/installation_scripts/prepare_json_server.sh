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

# Is needed for exectution
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
source ~/.bashrc