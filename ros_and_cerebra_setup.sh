# This script assumes 
#   - that Ubuntu Desktop 22.04.2 is installed
#   - the default-user "pib" is executing it. If not: change the DEFAULT_USER below accordingly
# The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...
# To run the script do the next steps:
# 1. sudo apt install curl
# 2. curl "https://raw.githubusercontent.com/pib-rocks/setup-pib/main/ros_and_cerebra_setup.sh" -O
# 3. chmod 755 ./ros_and_cerebra_setup.sh
# 4. ./ros_and_cerebra_setup.sh
#
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
ROS_CAMERA_NODE_LINK="https://github.com/pib-rocks/ros2_oak_d_lite/archive/refs/heads/master.zip"
ROS_CAMERA_NODE_DIR="$ROS_WORKING_DIR/ros_camera_node_dir"
ROS_CAMERA_NODE_ZIP="ros_camera_node.zip"
ROS_CEREBRA_BOOT_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot.sh"
ROS_CAMERA_NODE_BOOT_TEMPLATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_camera_boot_template.sh"
ROS_CEREBRA_BOOT_SERVICE_TEMPLATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_cerebra_boot_service_template.sh"
ROS_CAMERA_NODE_BOOT_SERVICE_TEMPLATE_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/ros_camera_boot_service_template.sh"
# Set the name of your database file
DATABASE_FILE="/var/lib/phpliteadmin/cerebra.db"
# Set the name of your SQL file
CEREBRA_INIT_QUERY_FILE="cerebra_init_query.sql"
CEREBRA_INIT_QUERY_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/cerebra_init_query.sql"
#
# Adding Universe repo, upgrading and installing basic packages
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get -y upgrade
# libusb-1.0-0 libudev1 procps are dependencies of later installed Tinkerforge brick-deamon
sudo apt-get install -y python3 python3-pip git curl software-properties-common unzip sqlite3 phpliteadmin locales libusb-1.0-0 libudev1 procps openssh-server
#
# Setting up ROS2
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "Adding ros2.list to repositories:"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> $USER_HOME/.bashrc
#
# Install rosbridge-server
echo 'Install rosbridge-server...'
sudo apt install -y ros-humble-rosbridge-server
#
# Installing TinkerForge software including Brick Daemon, Brick Viewer and Python API bindings
# Brick daemon
sudo apt-get install -y libusb-1.0-0 libudev1 procps
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
# Setting up the camera, including AI capabilities
# Depth-AI
sudo curl -sSL https://docs.luxonis.com/install_dependencies.sh | sudo bash
python3 -m pip install depthai
#Git examples for Depth-AI
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python
cd examples
python3 install_requirements.py
#Hand tracker
git clone https://github.com/geaxgx/depthai_hand_tracker.git
cd depthai_hand_tracker
python3 requirements.py
#
# Setup Cerebra
echo -e '\nInstall nginx...'
sudo apt install -y nginx
# If the 'html' directory inside of nginx doesn't exist, it will be created
if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then sudo -S mkdir -p $DEFAULT_NGINX_HTML_DIR; fi
echo -e '\nClean up the html directory...'
cd $DEFAULT_NGINX_HTML_DIR && sudo -S rm -r * 
cd $USER_HOME
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
# Install ros node for camera
echo -e '\nInstalling ros node for camera...'
curl $ROS_CAMERA_NODE_LINK -L --output $ROS_WORKING_DIR/$ROS_CAMERA_NODE_ZIP --create-dirs
sudo unzip $ROS_WORKING_DIR/$ROS_CAMERA_NODE_ZIP -d $ROS_CAMERA_NODE_DIR
rm $ROS_WORKING_DIR/$ROS_CAMERA_NODE_ZIP
cd $ROS_CAMERA_NODE_DIR
sudo colcon build
#
curl $CEREBRA_INIT_QUERY_LINK -L --output $ROS_WORKING_DIR/$CEREBRA_INIT_QUERY_FILE --create-dirs
# Create the database (if it doesn't exist) and initialize it with the SQL file
echo "Creating (if not exist) and initializing SQLite database $DATABASE_FILE with $ROS_WORKING_DIR/$CEREBRA_INIT_QUERY_FILE..."
sudo sqlite3 $DATABASE_FILE < $ROS_WORKING_DIR/$CEREBRA_INIT_QUERY_FILE
echo -e "\nDatabase initialized successfully!"
#
# Setup system to start Cerebra and ROS2 at boot time
# Create boot script for ros_bridge_server
curl $ROS_CEREBRA_BOOT_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot.sh
# Create boot script for ros_camera node
curl $ROS_CAMERA_NODE_BOOT_TEMPLATE_LINK -L --output $ROS_WORKING_DIR/ros_camera_boot_template.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_camera_boot_template.sh
sh $ROS_WORKING_DIR/ros_camera_boot_template.sh $ROS_CAMERA_NODE_DIR > $ROS_WORKING_DIR/ros_camera_boot.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_camera_boot.sh
rm $ROS_WORKING_DIR/ros_camera_boot_template.sh
# Create service which starts ros and cerebra by system boot
curl $ROS_CEREBRA_BOOT_SERVICE_TEMPLATE_LINK -L --output $ROS_WORKING_DIR/ros_cerebra_boot_service_template.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_cerebra_boot_service_template.sh
sh $ROS_WORKING_DIR/ros_cerebra_boot_service_template.sh $DEFAULT_USER $ROS_WORKING_DIR > $ROS_WORKING_DIR/ros_cerebra_boot.service
sudo mv $ROS_WORKING_DIR/ros_cerebra_boot.service /etc/systemd/system
rm $ROS_WORKING_DIR/ros_cerebra_boot_service_template.sh
# Create service which starts ros camera node by system boot
curl $ROS_CAMERA_NODE_BOOT_SERVICE_TEMPLATE_LINK -L --output $ROS_WORKING_DIR/ros_camera_boot_service_template.sh
sudo chmod 755 $ROS_WORKING_DIR/ros_camera_boot_service_template.sh
sh $ROS_WORKING_DIR/ros_camera_boot_service_template.sh $DEFAULT_USER $ROS_WORKING_DIR > $ROS_WORKING_DIR/ros_camera_boot.service
sudo mv $ROS_WORKING_DIR/ros_camera_boot.service /etc/systemd/system
rm $ROS_WORKING_DIR/ros_camera_boot_service_template.sh
# Enable new services
sudo systemctl daemon-reload
sudo systemctl enable ros_cerebra_boot.service
sudo systemctl enable ros_camera_boot.service
# Enable and start ssh server
sudo systemctl enable ssh --now
# Done! :-) Please restart to 
echo -e '\nCongratulations! The setup completed succesfully!'
echo -e '\nThe system will be restarted now'
sleep 5
sudo shutdown -r now
