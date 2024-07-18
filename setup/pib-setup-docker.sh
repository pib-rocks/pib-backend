
export RED_TEXT_COLOR="\e[31m"
export YELLOW_TEXT_COLOR="\e[33m"
export GREEN_TEXT_COLOR="\e[32m"
export CYAN_TEXT_COLOR="\e[36m"
export RESET_TEXT_COLOR="\e[0m"
export NEW_LINE="\n"

# Github repo origins and branches (branch values will be replaced in dev-mode)
export FRONTEND_REPO="https://github.com/pib-rocks/cerebra.git"
export BACKEND_REPO="https://github.com/pib-rocks/pib-backend.git"
export frontend_branch="main"
export backend_branch="main"

# Validate input parameters
export is_dev_mode=false
while [ $# -gt 0 ]; do
	case "$1" in
		# Assign default and feature branches for dev-mode
		-f=* | --frontendBranch=*)
			is_dev_mode=true
			frontend_branch="${1#*=}"
			;;
		-b=* | --backendBranch=*)
			is_dev_mode=true
			backend_branch="${1#*=}"
			;;
		-h | --help)
			show_help
			;;
		*)
			echo -e "$RED_TEXT_COLOR""Invalid input options. Here are some infos about the possible script inputs:""$RESET_TEXT_COLOR""$NEW_LINE"
			show_help
	esac
	shift
done


# Refresh the linux packages list (sometimes necessary for packages that are required in the installion scripts)
sudo apt update
# These packages are installed seperately, since the installation scripts are dependent on them
sudo apt-get install -y git curl


if [ "$is_dev_mode" = true ] 
then
	echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""-- Checking if user-specified branches exist --""$RESET_TEXT_COLOR""$NEW_LINE"

	if git ls-remote --exit-code --heads "$FRONTEND_REPO" "$frontend_branch" >/dev/null 2>&1; then
		echo -e "$CYAN_TEXT_COLOR""Frontend repo branch used: ""$RESET_TEXT_COLOR""$frontend_branch"
	else
		echo -e "$RED_TEXT_COLOR""Frontend repo: no branch called $frontend_branch was found""$RESET_TEXT_COLOR""$NEW_LINE"
		show_help
	fi

	if git ls-remote --exit-code --heads "$BACKEND_REPO" "$backend_branch" >/dev/null 2>&1; then
		echo -e "$CYAN_TEXT_COLOR""Backend repo branch used: ""$RESET_TEXT_COLOR""$backend_branch"
	else
		echo -e "$RED_TEXT_COLOR""Backend repo: no branch called $backend_branch was found""$RESET_TEXT_COLOR""$NEW_LINE"
		show_help
	fi

	echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Check for user-specified branches completed --""$RESET_TEXT_COLOR""$NEW_LINE"
fi


echo "Hello $USER! We start the setup by allowing you permanently to run commands with admin-privileges."
if [[ "$(id)" == *"(sudo)"* ]]; then
	echo "For this change please enter your password..."
	sudo bash -c "echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
else
	echo "For this change please enter the root-password. It is most likely just your normal one..."
	su root bash -c "usermod -aG sudo $USER ; echo '$USER ALL=(ALL) NOPASSWD:ALL' | tee /etc/sudoers.d/$USER"
fi


# Install Docker Engine
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl git
sudo install -y -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# allow communication between docker-host and the local x-server
export DISPLAY=":0.0"
sudo xhost +local:

# create a boot-service for setting permisssions for 'local:' on startup
cat > ~/xhost_enable_local.service << EOL
[Unit]
Description=Enables Connections to the host X-Server from within a docker container

[Service]
User=pib
Environment=DISPLAY=:0.0
Restart=on-failure
RestartSec=5s
ExecStart=/usr/bin/xhost +local:

[Install]
WantedBy=multi-user.target
EOL
sudo mv ~/xhost_enable_local.service /etc/systemd/xhost_enable_local.service
sudo chmod 700 /etc/systemd/xhost_enable_local.service
sudo systemctl enable /etc/systemd/xhost_enable_local.service

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin


echo -e "INSTALLED DOCKER ENGINE"

mkdir ~/app
git clone -b "$backend_branch" --recurse-submodules https://github.com/pib-rocks/pib-backend.git ~/app/pib-backend
git clone -b "$frontend_branch" --recurse-submodules https://github.com/pib-rocks/cerebra.git ~/app/cerebra
touch ~/app/pib-backend/password.env
echo -e "Pulled pib-backend and Cerebra repositories to ~/app"

cd ~/app/pib-backend
sudo docker compose --profile camera --profile motors --profile voice_assistant --profile display up -d

cd ~/app/cerebra
sudo docker compose up -d

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""Setup finished.""$RESET_TEXT_COLOR""$NEW_LINE"
echo -e "$NEW_LINE""$YELLOW_TEXT_COLOR""Don't forget to put your API keys in ~/app/pib-backend/password.env"
echo -e "To check docker container status use 'docker ps -a'"
echo -e "To stop docker containers, use 'docker compose down' in ~/app/cerebra or ~/app/pib-backend"
echo -e "To start docker containers used 'docker compose up -d' in ~/app/cerebra or ~/app/pib-backend"

# Doesn't require sudo
# newgrp kills script, that's why it's at the end
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker