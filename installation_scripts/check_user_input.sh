#!/bin/bash
#
# This script validates the user input arguments when calling the setup script
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

# Show infos about the command-line parameter options, then exit the script
help_function() 
{
	echo -e "Information about this script:"
	echo -e "This script has two execution modes (normal mode and development mode).""$NEW_LINE"
	echo -e "$YELLOW_TEXT_COLOR""To start the script in normal mode, don't add any arguments or options.""$RESET_TEXT_COLOR"
	echo -e "Example: ./setup-pib""$NEW_LINE"
	echo -e "$YELLOW_TEXT_COLOR""To start the script in development mode:""$RESET_TEXT_COLOR"
	echo -e "- add the -d option after the script name."
	echo -e "- specify the name of the default branch to be pulled as an argument"
	echo -e "- specify the name of the feature branch as the third argument"
	echo -e "$NEW_LINE""Example: ./setup-pib -d main PR-368"
    echo -e "Example: ./setup-pib -d develop PR-368"
    exit "$INPUT_OUTPUT_ERROR_STATUS"
}

echo -e "$YELLOW_TEXT_COLOR""-- Checking possible user input options and arguments --""$RESET_TEXT_COLOR""$NEW_LINE"

# Check if script was started in dev-mode (case insensitive comparison)
if [ "$FIRST_USER_INPUT" = "-d" ] || [ "$FIRST_USER_INPUT" = "-D" ] || [ "$FIRST_USER_INPUT" = "dev" ] 
then
	is_dev_mode="$TRUE"

	# Assign the second and thrid input argument to variables, if none are null
	if [[ -n "$SECOND_USER_INPUT" ]] && [[ -n "$THIRD_USER_INPUT" ]]
	then
		user_default_branch="$SECOND_USER_INPUT"
		user_feature_branch="$THIRD_USER_INPUT"
	else
		echo -e "$RED_TEXT_COLOR""Invalid argument syntax. Here is some info about the possible user inputs:""$RESET_TEXT_COLOR""$NEW_LINE"
		help_function
	fi

# Show help if any other option was called
elif [ -n "$FIRST_USER_INPUT" ] 
then
	echo -e "$RED_TEXT_COLOR""Invalid option inputs. Here is some info about the possible user inputs:""$RESET_TEXT_COLOR""$NEW_LINE"
	help_function
fi

echo -e "$GREEN_TEXT_COLOR""-- User input option and argument syntax valid --""$RESET_TEXT_COLOR""$NEW_LINE"