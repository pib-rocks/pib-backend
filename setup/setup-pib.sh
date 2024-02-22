#!/bin/bash
#
# This script installs all necessary software and sets required configurations for running pib
#
# It assumes:
#   - that Ubuntu Desktop 22.04 is installed
#   - the default-user "pib" is executing it
#

# Exported variables for all subshells: Codes for "echo -e" output text formatting
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export CYAN_TEXT_COLOR="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Exported variables for all subshells: Exit codes for error detection
export INPUT_OUTPUT_ERROR_STATUS=5
export FAILED_CHECK_STATUS=255

# Exported variables for all subshells: Boolean constants for checks
export TRUE="true"
export FALSE="false"

# Default ubuntu paths
export DEFAULT_USER="pib"
export USER_HOME="/home/$DEFAULT_USER"
export ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
mkdir "$ROS_WORKING_DIR"

# Create temporary directory for installation files
export TEMPORARY_SETUP_DIR=""
TEMPORARY_SETUP_DIR=$(mktemp --directory /var/tmp/pib-temp.XXX)
export BACKEND_DIR="$TEMPORARY_SETUP_DIR/backend"
export FRONTEND_DIR="$TEMPORARY_SETUP_DIR/frontend"
export SETUP_DIR="$BACKEND_DIR/setup"
export SETUP_FILES="$TEMPORARY_SETUP_DIR/setup/setup_files"

# Variables for user input options
export user_default_branch=""
export user_feature_branch=""
export is_dev_mode="$FALSE"

# Origins of our github repositories
export FRONTEND_REPO="https://github.com/pib-rocks/cerebra.git"
export BACKEND_REPO="https://github.com/pib-rocks/pib-backend.git"

# Create an associative array (=map). This will be filled with repo-origin branch-name pairs
declare -A repo_map
repo_map["$FRONTEND_REPO"]="main"
repo_map["$BACKEND_REPO"]="main"

show_help() 
{
	echo -e "setup-pib.sh input help:"
	echo -e "This script has two execution modes (normal mode and development mode).""$NEW_LINE"
	echo -e "$YELLOW_TEXT_COLOR""To start the script in normal mode, don't add any arguments or options.""$RESET_TEXT_COLOR"
	echo -e "Example: ./setup-pib""$NEW_LINE"
	echo -e "$YELLOW_TEXT_COLOR""Starting the script in development mode:""$RESET_TEXT_COLOR"
	echo -e "- Default branch parameter -"
	echo -e "-d=YourBranchName or --defaultBranch=YourBranchName"
	echo -e "$CYAN_TEXT_COLOR""(This branch will be checked out if the feature branch wasn't found)""$RESET_TEXT_COLOR"
	echo -e "$NEW_LINE""- Feature branch parameter -"
	echo -e "-f=YourBranchName or --featureBranch=YourBranchName"
	echo -e "$CYAN_TEXT_COLOR""(If a branch with this name can be found in a repo, it will be checked out instead of the default branch)""$RESET_TEXT_COLOR"
	echo -e "$NEW_LINE""Dev-mode examples:"
	echo -e "    ./setup-pib -d=main -f=PR-368"
    echo -e "    ./setup-pib --defaultBranch=main --featureBranch=PR-368"
    exit "$INPUT_OUTPUT_ERROR_STATUS"
}

echo -e "$YELLOW_TEXT_COLOR""-- Checking possible user input options and arguments --""$RESET_TEXT_COLOR""$NEW_LINE"

# Iterate through all user input parameters
while [ $# -gt 0 ]; do
	case "$1" in
		# Assign default and feature branches for dev-mode
		-d=* | --defaultBranch=*)
			is_dev_mode="$TRUE"
			user_default_branch="${1#*=}"
			;;
		-f=* | --featureBranch=*)
			is_dev_mode="$TRUE"
			user_feature_branch="${1#*=}"
			;;
		-h | --help)
			show_help
			;;
		*)
			echo -e "$RED_TEXT_COLOR""Invalid option inputs. Here is some info about the possible user inputs:""$RESET_TEXT_COLOR""$NEW_LINE"
			show_help
	esac
	shift
done

# We want the user pib to setup things without password (sudo without password)
# Yes, we are aware of the security-issues..
echo "Hello pib! We start the setup by allowing you permanently to run commands with admin-privileges."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $DEFAULT_USER ; echo '$DEFAULT_USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$DEFAULT_USER"
fi

# Redirect console output to a log file
LOG_FILE="$USER_HOME/setup-pib.log"
exec > >(tee -a "$LOG_FILE") 2>&1

# Refresh the linux packages list (sometimes necessary for packages that are required in the installion scripts)
sudo apt update
# These packages are installed seperately, since the installation scripts are dependent on them
sudo apt-get install -y git curl

# In dev-mode check if the specified branches exist for each repo
if [ "$is_dev_mode" = "$TRUE" ] 
then

	# Iterate through all repos
	for repo in "${!repo_map[@]}" ; do
	
		if git ls-remote --exit-code --heads "$repo" "$user_feature_branch" >/dev/null 2>&1; then
			repo_map["$repo"]="$user_feature_branch"
		elif git ls-remote --exit-code --heads "$repo" "$user_default_branch" >/dev/null 2>&1; then
			repo_map["$repo"]="$user_default_branch"
		else
			echo -e "$RED_TEXT_COLOR""Neither $user_feature_branch nor $user_default_branch exists in the $repo repository.""$RESET_TEXT_COLOR""$NEW_LINE"
			show_help
		fi
	done
fi

# clone repos
git clone -b "${repo_map[$BACKEND_REPO]}" "$BACKEND_REPO" "$BACKEND_DIR"
git clone -b "${repo_map[$FRONTEND_REPO]}" "$FRONTEND_REPO" "$FRONTEND_DIR"


# The following scripts are sourced into the same shell as this script,
# allowing them to acces all variables and context
# Check system variables
source "$SETUP_DIR/installation_scripts/check_system_variables.sh"
# Install system packages
source "$SETUP_DIR/installation_scripts/install_system_packages.sh"
# Install python packages
source "$SETUP_DIR/installation_scripts/install_python_packages.sh"
# Install tinkerforge
source "$SETUP_DIR/installation_scripts/install_tinkerforge.sh"
# Install Cerebra
source "$SETUP_DIR/installation_scripts/install_cerebra.sh"
# Install pib ros-packages
source "$SETUP_DIR/installation_scripts/setup_packages.sh"
# Adjust system settings
source "$SETUP_DIR/installation_scripts/set_system_settings.sh"

# install update-pip
cp "$SETUP_DIR/update-pib.sh" "$USER_HOME/update-pib.sh"
sudo chmod 777 ~/update-pib.sh

# Get ros_config
cp "$SETUP_FILES/ros_config.sh" "$ROS_WORKING_DIR/ros_config.sh"

# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
cp "$SETUP_FILES/ros_cerebra_boot.sh" "$ROS_WORKING_DIR/ros_cerebra_boot.sh"
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh

# Create service which starts ros and cerebra by system boot
cp "$SETUP_FILES/ros_cerebra_boot.service" "$ROS_WORKING_DIR/ros_cerebra_boot.service"
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system

# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now

# Download animated pib eyes
cp "$SETUP_FILES/pib-eyes-animated.gif" "$USER_HOME/Desktop/pib-eyes-animated.gif"

# Move log file to temporary setup folder
mv "$LOG_FILE" "$TEMPORARY_SETUP_DIR"

echo -e "$NEW_LINE""Congratulations! The setup completed succesfully!"
echo -e "$NEW_LINE""Please restart the system to apply changes..."