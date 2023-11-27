#!/bin/bash
#
# Create a .gitmodules file based on user input

# Github repo origin links
readonly SETUP_PIB_ORIGIN="https://github.com/pib-rocks/setup-pib.git"
readonly PIB_API_ORIGIN="https://github.com/pib-rocks/pib-api.git"

readonly ROS_PACKAGES_ORIGIN="https://github.com/pib-rocks/ros-packages.git"
readonly DATATYPES_ORIGIN="https://github.com/pib-rocks/datatypes.git"
readonly MOTORS_ORIGIN="https://github.com/pib-rocks/motors.git"
readonly OAK_D_LITE_ORIGIN="https://github.com/pib-rocks/ros2_oak_d_lite.git"
readonly VOICE_ASSISTANT_ORIGIN="https://github.com/pib-rocks/voice-assistant.git"

# Default branch options
readonly MAIN_BRANCH="main"
readonly MAIN_ALTERNATIVE_BRANCH="master"

# Assign default branches for the different pib system components
setup_pib_branch_name="$MAIN_BRANCH"
database_branch_name="$MAIN_BRANCH"
pib_api_branch_name="$MAIN_BRANCH"

# Assign default branches for our ros packages
ros_packages_branch_name="$MAIN_BRANCH"
motors_branch_name="$MAIN_BRANCH"
datatypes_branch_name="$MAIN_BRANCH"
oak_d_lite_branch_name="$MAIN_ALTERNATIVE_BRANCH"
voice_assistant_branch_name="$MAIN_BRANCH"

# Create an associative array, with repo-origin, branch-name pairs
declare -A repo_map
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
	echo -e "$YELLOW_TEXT_COLOR""Checking if the specified branches exist...""$RESET_TEXT_COLOR"		

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

			# Check edge case if main branch in user_default, but the branch is named "master". TODO: Remove this check once oak_d_lite branch is renamed
			if [ "$user_default_branch" = "$MAIN_BRANCH" ] && [ "${repo_map[$repo_origin]}" = "$MAIN_ALTERNATIVE_BRANCH" ]
			then
				break
			fi

			echo -e "$RED_TEXT_COLOR""No feature or default branch was found for (at least) this repo: ""$NEW_LINE""$repo_origin""$RESET_TEXT_COLOR"
			exit 1
		fi
	done
fi

# Print key and value of associative array 
echo -e "$NEW_LINE""These branches were found for each repo:"
for repo_origin in "${!repo_map[@]}" ; do 
	echo -e "$YELLOW_TEXT_COLOR""repo: ""$RESET_TEXT_COLOR""$repo_origin" "$YELLOW_TEXT_COLOR""branch: ""$RESET_TEXT_COLOR""${repo_map[$repo_origin]}"
done 
echo -e "$NEW_LINE"

# Error handling if no feature branch was found in dev mode
if  [ "$is_dev_mode" = "$TRUE" ] && [ "$at_least_one_feature_branch_found" = "$FALSE" ]
then
	echo -e "$RED_TEXT_COLOR""The specified feature branch wasn't found in any repo. Exiting script.""$RESET_TEXT_COLOR"
	exit 1
fi

# Path-Variables
readonly DEFAULT_USER="pib"
readonly USER_HOME="/home/$DEFAULT_USER"
readonly ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
readonly GIT_PROJECT_DIR="$ROS_WORKING_DIR""/src/"

# Setup ros-workspace
mkdir "$ROS_WORKING_DIR"
mkdir src
sudo chmod -R 777 "$ROS_WORKING_DIR"
sudo chmod -R 777 "$GIT_PROJECT_DIR"

# Initialize a git repo
echo 'check if git init is done'
cd "$GIT_PROJECT_DIR"
if [ ! -f .git ]; then
	git init
fi

# Git pull ros-packages repo
git pull https://github.com/pib-rocks/ros-packages.git

# Folder names (=package names) for submodules
readonly VOICE_ASSISTANT_PACKAGE_NAME="voice-assistant"
readonly MOTORS_PACKAGE_NAME="motors"
readonly DATATYPES_PACKAGE_NAME="datatypes"
readonly OAK_D_LITE_PACKAGE_NAME="ros2_oak_d_lite"

# Assemble submodule information in variables (\n = new line \t = tab)
readonly VOICE_ASSISTANT_GITMODULE="[submodule \"$VOICE_ASSISTANT_PACKAGE_NAME\"]\n\t path = $VOICE_ASSISTANT_PACKAGE_NAME\n\t url = $VOICE_ASSISTANT_ORIGIN\n\t branch = ${repo_map[$VOICE_ASSISTANT_ORIGIN]}\n"
readonly MOTORS_GITMODULE="[submodule \"$MOTORS_PACKAGE_NAME\"]\n\t path = $MOTORS_PACKAGE_NAME\n\t url = $MOTORS_ORIGIN\n\t branch = ${repo_map[$MOTORS_ORIGIN]}\n"
readonly DATATYPES_GITMODULE="[submodule \"$DATATYPES_PACKAGE_NAME\"]\n\t path = $DATATYPES_PACKAGE_NAME\n\t url = $DATATYPES_ORIGIN\n\t branch = ${repo_map[$DATATYPES_ORIGIN]}\n"
readonly OAK_D_LITE_GITMODULE="[submodule \"$OAK_D_LITE_PACKAGE_NAME\"]\n\t path = $OAK_D_LITE_PACKAGE_NAME\n\t url = $OAK_D_LITE_ORIGIN\n\t branch = ${repo_map[$OAK_D_LITE_ORIGIN]}\n"

# Overwrite the .gitmodules file
chmod 755 "$GIT_PROJECT_DIR"".gitmodules"
readonly GITMODULE_FILE_CONTENT="$VOICE_ASSISTANT_GITMODULE""$MOTORS_GITMODULE""$DATATYPES_GITMODULE""$OAK_D_LITE_GITMODULE"
echo -e $GITMODULE_FILE_CONTENT > "$GIT_PROJECT_DIR"".gitmodules"

echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR"".gitmodules file creation completed""$RESET_TEXT_COLOR"

exit 0