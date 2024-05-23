#!/bin/bash
#
# This script checks if the username and ubuntu version is matching the expected values
# To properly run this script relies on being sourced by the "setup-pib.sh"-script
#
# Block Time Measuring
start_time=$(date +%s)

echo -e "$YELLOW_TEXT_COLOR""-- Checking system version and username --""$RESET_TEXT_COLOR""$NEW_LINE"

readonly EXPECTED_USERNAME="$DEFAULT_USER"
readonly EXPECTED_UBUNTU_VERSION="22.04"

# Check if this script is run by the expected username
if [ "$(whoami)" != "$EXPECTED_USERNAME" ]; then
        echo -e "$RED_TEXT_COLOR""This script must be run as user: ""$EXPECTED_USERNAME""$RESET_TEXT_COLOR"
        exit "$FAILED_CHECK_STATUS"
fi

# Check on the right Ubuntu version
if [ "$(lsb_release -rs)" != "$EXPECTED_UBUNTU_VERSION" ]; then
	echo -e "$RED_TEXT_COLOR""Incompatible Ubuntu version detected. Cerebra requires Ubuntu 22.04.""$RESET_TEXT_COLOR"
	echo 'Do you still want to continue the installation (installation might fail)? [Yes/No]'
	while true; do
		read CONTINUE
		if [[ ${CONTINUE,,} == "yes" ]]; then
			break
		elif [[ ${CONTINUE,,} == "no" ]]; then
		        echo "Setup has been cancelled"
		        exit "$FAILED_CHECK_STATUS"
		else
			echo "Your input is not correct"
			continue
		fi
	done
fi

sleep 2

end_time=$(date +%s)
elapsed_time=$(( end_time - start_time ))

echo -e "$GREEN_TEXT_COLOR""-- System version and username check completed --""$RESET_TEXT_COLOR""$NEW_LINE"
echo "<Elapsed time: $elapsed_time seconds> [check_system_variables.sh]"