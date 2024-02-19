#!/bin/bash
#
# This script downloads the pib-setup repo and unzips it to a temporary directory
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Downloading setup files from github --""$RESET_TEXT_COLOR""$NEW_LINE"

SETUP_ARCHIVE_URL="https://github.com/pib-rocks/setup-pib/archive/refs/heads/""$SETUP_PIB_BRANCH"".zip"
SETUP_ARCHIVE_NAME="setup-pib-""$SETUP_PIB_BRANCH"".zip"

# Download the zipped repo and unpack it in the temporary folder
curl "$SETUP_ARCHIVE_URL" --location --output "$SETUP_ARCHIVE_NAME" 
mv "$SETUP_ARCHIVE_NAME" "$TEMPORARY_SETUP_DIR"
unzip "$TEMPORARY_SETUP_DIR/""$SETUP_ARCHIVE_NAME" -d "$TEMPORARY_SETUP_DIR"

installation_files_dir="$TEMPORARY_SETUP_DIR/""setup-pib-""$SETUP_PIB_BRANCH""/installation_scripts"

# Change permissions for all files in temp folder
chmod --recursive 775 "$TEMPORARY_SETUP_DIR"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Setup file download completed --""$RESET_TEXT_COLOR""$NEW_LINE"