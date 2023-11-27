#!/bin/bash
#
# This script checks if the username and ubuntu version is matching the expected values

readonly EXPECTED_USERNAME="pib"
readonly EXPECTED_UBUNTU_VERSION="22.04"

# Check if this script is run by the expected username
if [ "$(whoami)" != "$EXPECTED_USERNAME" ]; then
        echo -e "$red_text_color""This script must be run as user: ""$EXPECTED_USERNAME""$reset_text_color"
        exit "$FAILED_CHECK_STATUS"
fi

# Check on the right Ubuntu version
if [ "$(lsb_release -rs)" != "$EXPECTED_UBUNTU_VERSION" ]; then
	echo -e "$red_text_color""This Ubuntu version is incompatible with Cerebra!""$reset_text_color"
	echo 'Do you still want to continue the installation? [Yes/No]'
	while true; do
		read CONTINUE
		if [[ ${CONTINUE,,} == "yes" ]]; then
			break
		elif [[ ${CONTINUE,,} == "no" ]]; then
		        echo "Setup has been stoped"
		        exit "$FAILED_CHECK_STATUS"
		else
			echo "Your input is not correct"
			continue
		fi
	done
fi

echo -e "$green_text_color""You're using the correct ubuntu version and ubuntu username!""$reset_text_color""$new_line"
exit "$SUCCESS_STATUS"