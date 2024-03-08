#!/bin/bash
#
# This script prepare everything for the JSON-Server
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Prepare JSON-Server --""$RESET_TEXT_COLOR"

PIB_FOLDER=$(find /var/tmp -type d -name 'pib-temp.*' -print -quit)

echo "Folder with the folowing name found: " + $PIB_FOLDER

pwd

mkdir json-server

cp $PIB_FOLDER/frontend/server $USER_HOME/json-server
cp $PIB_FOLDER/frontend/package.json $USER_HOME/json-server
cp $PIB_FOLDER/frontend/package-lock.json $USER_HOME/json-server

