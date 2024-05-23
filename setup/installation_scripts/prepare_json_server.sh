#!/bin/bash
#
# This script prepares everything for the JSON-Server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script
#
# Block Time Measuring
start_time=$(date +%s)

echo -e "$YELLOW_TEXT_COLOR""-- Prepares JSON-Server --""$RESET_TEXT_COLOR"

PIB_FOLDER=$(find /var/tmp -type d -name 'pib-temp.*' -print -quit)

echo "Folder with the folowing name found: " + $PIB_FOLDER

mkdir json-server

cp -r $PIB_FOLDER/frontend/server $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package.json $ROS_WORKING_DIR/json-server
cp $PIB_FOLDER/frontend/package-lock.json $ROS_WORKING_DIR/json-server

# Nvm is already installed unter /etc/nvm 
sed -i 's/export NVM_DIR="$HOME\/.nvm/export NVM_DIR="\/etc\/nvm/g' ~/.bashrc
source ~/.bashrc

sleep 2

end_time=$(date +%s)
elapsed_time=$(( end_time - start_time ))

echo "<Elapsed time: $elapsed_time seconds> [prepare_json_server.sh]"