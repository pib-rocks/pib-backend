#!/bin/bash
#
# This script assumes:
#   - that Ubuntu Desktop 22.04.2 is installed
#   - the default-user "pib" is executing it
#
UBUNTU_VERSION=$(lsb_release -rs)
DEFAULT_USER="pib"
USER_HOME="/home/$DEFAULT_USER"
ROS_WORKING_DIR="$USER_HOME/ros_working_dir"
DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
NGINX_CONF_FILE="nginx.conf"
NGINX_CONF_FILE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/nginx.conf"
#
CEREBRA_ARCHIVE_URL_PATH="https://pib.rocks/wp-content/uploads/pib_data/cerebra-latest.zip"
CEREBRA_ARCHIVE_NAME="cerebra-latest.zip"
#
ROS_CONFIG_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_config.sh"
#
ROS_CEREBRA_BOOT_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot.sh"
ROS_CEREBRA_BOOT_SERVICE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot.service"
#
PHPLITEADMIN_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"
DATABASE_DIR="$USER_HOME/pib_data"
DATABASE_FILE="pibdata.db"
DATABASE_INIT_QUERY_FILE="cerebra_init_database.sql"
DATABASE_INIT_QUERY_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/cerebra_init_database.sql"
#
#
PIB_API_DIR="$USER_HOME/flask"
PIB_API_URL_PATH="https://github.com/pib-rocks/pib-api/archive/refs/heads/main.zip"
#
ROS_PACKAGES_LINK="https://raw.githubusercontent.com/pib-rocks/ros-packages/main/packages-set-up.sh"
ROS_UPDATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/update-pib.sh"
#
# We make sure that this script is run by the user "pib"
if [ "$(whoami)" != "pib" ]; then
        echo "This script must be run as user: pib"
        exit 255
fi
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
#Check on the right Ubuntu version
if [ ! $UBUNTU_VERSION == 22.04 ]; then
	echo 'Your Ubuntu version is not tested or predicted for Cerebra!'
	echo 'Do you still want to continue the installation? [Yes/No]'
	while true; do
		read CONTINUE
		if [[ ${CONTINUE,,} == "yes" ]]; then
			break
		elif [[ ${CONTINUE,,} == "no" ]]; then
		        echo "Setup has been stoped"
		        exit 255
		else
			echo "Your input is not correct"
			continue
		fi
	done
fi
#
# Adding Universe repo, upgrading and installing basic packages
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get -y upgrade
# libusb-1.0-0 libudev1 procps are dependencies of later installed Tinkerforge brick-deamon
sudo apt-get install -y python3 python3-pip git curl openssh-server software-properties-common unzip sqlite3 locales libusb-1.0-0 libudev1 procps php8.1-fpm php-sqlite3
#
# Setting up ROS2
explain this: sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "Adding ros2.list to repositories..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> $USER_HOME/.bashrc
sudo apt-get install colcon
echo 'source /home/pib/ros_working_dir/install/setup.bash' >> $USER_HOME/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> $USER_HOME/.bashrc
#Install for voice assistant
#
pip install --upgrade openai
pip install google-cloud-texttospeech
pip install --upgrade google-cloud-speech
sudo apt install -y python3-pyaudio
pip install SpeechRecognition
#
# Install rosbridge-server
echo 'Install rosbridge-server...'
sudo apt install -y ros-humble-rosbridge-server
#
# Installing TinkerForge software including Brick Daemon, Brick Viewer and Python API bindings
# Brick daemon
PLATFORM_TYPE=$(uname -m)
if [ $PLATFORM_TYPE != 'aarch64' ]; then
	echo "Installing Brick daemon for $PLATFORM_TYPE"
	curl -O https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_amd64.deb
	sudo dpkg -i brickd_linux_latest_amd64.deb
else
	echo "Installing Brick daemon for $PLATFORM_TYPE"
	curl -O https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_arm64.deb
	sudo dpkg -i brickd_linux_latest_arm64.deb
fi
# Brick viewer
sudo apt-get install -y python3-pyqt5 python3-pyqt5.qtopengl python3-serial python3-tz python3-tzlocal
curl -O https://download.tinkerforge.com/tools/brickv/linux/brickv_linux_latest.deb
sudo dpkg -i brickv_linux_latest.deb
# Tinkerforge python APIs
curl -sSL https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.gpg > /dev/null
echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
sudo apt-get update
sudo apt-get install -y python3-tinkerforge
#
# Setup Cerebra
echo -e '\nInstall nginx...'
sudo apt install -y nginx
# If the 'html' directory inside of nginx doesn't exist, it will be created
if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then sudo -S mkdir -p $DEFAULT_NGINX_HTML_DIR; fi
echo -e '\nClean up the html directory...'
cd $DEFAULT_NGINX_HTML_DIR && sudo -S rm -r *
cd $USER_HOME
mkdir $ROS_WORKING_DIR
# Download Cerebra artifact to the working directory
echo -e '\nDownloading Cerebra application'
curl $CEREBRA_ARCHIVE_URL_PATH -L --output $ROS_WORKING_DIR/$CEREBRA_ARCHIVE_NAME
#
# Unzip cerebra files to nginx
echo -e '\nUnzip cerebra...'
#cd $RASP_TMP_FOLDER
sudo unzip $ROS_WORKING_DIR/$CEREBRA_ARCHIVE_NAME -d $DEFAULT_NGINX_HTML_DIR
#
# Setting up nginx to serve Cerebra locally
echo -e '\nDownloading nginx configuration file...'
sudo curl $NGINX_CONF_FILE_URL --output $DEFAULT_NGINX_DIR/$NGINX_CONF_FILE
#
# create src directory for all ros packages
cd $ROS_WORKING_DIR
mkdir src
#
# Install and configure phpLiteAdmin
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
curl $PHPLITEADMIN_LINK -L --output $ROS_WORKING_DIR/$PHPLITEADMIN_ZIP
sudo mkdir $PHPLITEADMIN_INSTALLATION_DIR
sudo chown -R www-data:www-data $PHPLITEADMIN_INSTALLATION_DIR
sudo chmod -R 755 $PHPLITEADMIN_INSTALLATION_DIR
sudo unzip $ROS_WORKING_DIR/$PHPLITEADMIN_ZIP -d $PHPLITEADMIN_INSTALLATION_DIR
sudo systemctl restart php8.1-fpm
# Create the database (if it doesn't exist) and initialize it with the SQL file
curl $DATABASE_INIT_QUERY_LINK -L --output $ROS_WORKING_DIR/$DATABASE_INIT_QUERY_FILE
echo "Creating (if not exist) and initializing SQLite database $DATABASE_FILE with $ROS_WORKING_DIR/$DATABASE_INIT_QUERY_FILE..."
mkdir $DATABASE_DIR
sudo chmod 777 $USER_HOME
sudo chmod 777 $DATABASE_DIR
sudo sqlite3 $DATABASE_DIR/$DATABASE_FILE < $ROS_WORKING_DIR/$DATABASE_INIT_QUERY_FILE
sudo chmod 766 $DATABASE_DIR/$DATABASE_FILE
echo -e "\nDatabase initialized successfully!"
# Create pib-api
echo "export PYTHONIOENCODING=utf-8" >> $USER_HOME/.bashrc
pip3 install pipenv
cd $USER_HOME
wget -O flask-api.zip $PIB_API_URL_PATH
unzip flask-api.zip
mv $USER_HOME/pib-api-main/flask/ $USER_HOME
sudo mv $USER_HOME/flask/pib_api_boot.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pib_api_boot.service
cd $USER_HOME
rm flask-api-zip
rm -r pib-api-main
# Allow editing in all src-directories
sudo chmod -R 777 $ROS_WORKING_DIR
sudo chmod -R 777 $ROS_WORKING_DIR/src
# install all ros-packages
cd $USER_HOME
wget -O package_set_up.sh $ROS_PACKAGES_LINK
chmod +x package_set_up.sh
yes | ./package_set_up.sh
# install update-pip
if [ -f $USER_HOME/update-pib.sh ]; then
  sudo rm update-pib.sh
fi
wget -O update-pib.sh $ROS_UPDATE_LINK
sudo chmod 777 update-pib.sh
echo "if [ -f /home/pib/update-pib.sh ]; then
        alias update-pib='/home/pib/update-pib.sh'
      fi
" >> $USER_HOME/.bashrc
# set permissions
cd $ROS_WORKING_DIR
colcon build
sudo chmod -R 777 $ROS_WORKING_DIR/build
sudo chmod -R 777 $ROS_WORKING_DIR/install
sudo chmod -R 777 $ROS_WORKING_DIR/log
# Get config
curl $ROS_CONFIG_LINK -L --output $ROS_WORKING_DIR/ros_config.sh
#
# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
curl $ROS_CEREBRA_BOOT_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh
# Create service which starts ros and cerebra by system boot
curl $ROS_CEREBRA_BOOT_SERVICE_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system
#
# Clean-up: remove unnecessary .zip directories
rm -r phpliteadmin_v1_9_9_dev.zip
rm -r cerebra-latest.zip
# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now
# Done! :-) Please restart to
echo -e '\nCongratulations! The setup completed succesfully!'
echo -e '\nPlease restart the system to apply changes...'
