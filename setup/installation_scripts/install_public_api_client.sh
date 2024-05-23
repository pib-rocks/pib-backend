#!/bin/bash
#
# This script installs the client for the public-api
# To properly run this script relies on being sourced by the "setup-pib.sh"-script
#
# Block Time Measuring
start_time=$(date +%s)

echo -e "$YELLOW_TEXT_COLOR""-- Install Public-Api-Client --""$RESET_TEXT_COLOR"

pip install "$BACKEND_DIR/public_api_client"
mkdir "$USER_HOME/public_api"
printf "{\n\t\"trybUrlPrefix\": \"\",\n\t\"publicApiToken\": \"\"\n}\n" > "$USER_HOME/public_api/config.json"

sleep 2

end_time=$(date +%s)
elapsed_time=$(( end_time - start_time ))

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Public-Api-Client installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"
echo "<Elapsed time: $elapsed_time seconds> [install_public_api_client.sh]"