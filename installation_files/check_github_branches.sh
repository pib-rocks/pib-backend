#!/bin/bash
#
# This script checks if there are existing github branches to match the user input
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Checking if the specified branches exist --""$RESET_TEXT_COLOR"		

# Default branch option
readonly MAIN_BRANCH="main"

# Assign default branches for the different pib system components
setup_pib_branch_name="$MAIN_BRANCH"
database_branch_name="$MAIN_BRANCH"
pib_api_branch_name="$MAIN_BRANCH"

# Assign default branches for our ros packages
ros_packages_branch_name="$MAIN_BRANCH"
motors_branch_name="$MAIN_BRANCH"
datatypes_branch_name="$MAIN_BRANCH"
oak_d_lite_branch_name="$MAIN_BRANCH"
voice_assistant_branch_name="$MAIN_BRANCH"

# Create an associative array, with repo-origin, branch-name pairs 
# Origin Variables are located in the main setup-pib script
repo_map["$SETUP_PIB_ORIGIN"]="$setup_pib_branch_name"
repo_map["$PIB_API_ORIGIN"]="$pib_api_branch_name"
repo_map["$ROS_PACKAGES_ORIGIN"]="$ros_packages_branch_name"
repo_map["$MOTORS_ORIGIN"]="$motors_branch_name"
repo_map["$DATATYPES_ORIGIN"]="$datatypes_branch_name"
repo_map["$OAK_D_LITE_ORIGIN"]="$oak_d_lite_branch_name"
repo_map["$VOICE_ASSISTANT_ORIGIN"]="$voice_assistant_branch_name"

# Define the order in which branches are tried to be set
declare -a branch_order
branch_order[0]="$user_feature_branch"
branch_order[1]="$user_default_branch"
at_least_one_feature_branch_found="$FALSE"

# Github branch exit-codes
readonly BRANCH_FOUND=0
readonly BRANCH_NOT_FOUND=2

# In dev-mode check if the branches exist on each repo
if [ "$is_dev_mode" = "$TRUE" ] 
then

	# Iterate through all repos
	for repo_origin in "${!repo_map[@]}" ; do
		
		# Check through all branches on the repo in the specified order until a match is found
		for branch in "${branch_order[@]}" ; do
		
			git ls-remote --exit-code --heads "${repo_origin}" "$branch" >/dev/null 2>&1
			EXIT_CODE=$?

			# If a branch was found, save it's name in the array
			if [ "$EXIT_CODE" -eq "$BRANCH_FOUND" ] 
			then
				repo_map["${repo_origin}"]="$branch"
				if [ "$branch" = "$user_feature_branch" ]; then at_least_one_feature_branch_found="$TRUE"; fi
				break
			fi
		done

		# If neither feature nor default branch was found, throw error and exit script
		if [ "${repo_map[$repo_origin]}" != "$user_feature_branch" ] && [ "${repo_map[$repo_origin]}" != "$user_default_branch" ] 
		then
			echo -e "$RED_TEXT_COLOR""No feature or default branch was found for (at least) this repo: ""$NEW_LINE""$repo_origin""$RESET_TEXT_COLOR"
			exit "$FAILED_SUBSCRIPT_STATUS"
		fi
	done
fi

# Print key and value of associative array 
echo -e "$NEW_LINE""These branches were found for each repo:"
for repo_origin in "${!repo_map[@]}" ; do 
	echo -e "$YELLOW_TEXT_COLOR""repo: ""$RESET_TEXT_COLOR""$repo_origin" "$YELLOW_TEXT_COLOR""branch: ""$RESET_TEXT_COLOR""${repo_map[$repo_origin]}"
done 

# Error handling if no feature branch was found in dev mode
if  [ "$is_dev_mode" = "$TRUE" ] && [ "$at_least_one_feature_branch_found" = "$FALSE" ]
then
	echo -e "$RED_TEXT_COLOR""The specified feature branch wasn't found in any repo. Exiting script.""$RESET_TEXT_COLOR"
	exit "$FAILED_SUBSCRIPT_STATUS"
fi

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Branch checking completed --""$RESET_TEXT_COLOR""$NEW_LINE"