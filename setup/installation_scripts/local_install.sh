#!/bin/bash

PIB_API_DIR="$HOME/flask"
PIB_API_SETUP_DIR="$BACKEND_DIR/pib_api"

ROS_WORKING_DIR="$HOME/ros_working_dir"
ROS_CAMERA_BOOT_DIR="$ROS_WORKING_DIR/src/camera/boot_scripts"
ROS_MOTORS_BOOT_DIR="$ROS_WORKING_DIR/src/motors/boot_scripts"
ROS_VOICE_ASSISTANT_BOOT_DIR="$ROS_WORKING_DIR/src/voice_assistant/boot_scripts"
ROS_PROGRAMS_BOOT_DIR="$ROS_WORKING_DIR/src/programs/boot_scripts"

DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
PYTHON_CODE_PATH="$HOME/cerebra_programs"

PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"
SETUP_FILES="$BACKEND_DIR/setup/setup_files"


PIB_BLOCKLY_SETUP_DIR="$BACKEND_DIR/pib_blockly"
PIB_BLOCKLY_SERVER_DIR="$HOME/pib_blockly_server"


# Install ROS2 Humble, rosbridge and colcon
function install_ros() {
  print INFO "Installing ROS"

  # Check that locale supports UTF-8 (should be pretty much always be the case with Ubuntu 22.04)
  if ! locale | grep -q 'UTF-8'; then
    print INFO "locale does not support UTF-8; switching to en_US.UTF-8"
    sudo apt -qq update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
  fi

  sudo apt -qq  install -y software-properties-common && \
  sudo add-apt-repository -y universe && \
  sudo apt -qq update && \
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  print INFO "Setup ROS sources"

  sudo apt -qq  update && \
  sudo apt -y -qq  upgrade

  sudo apt -qq install -y ros-humble-ros-base ros-dev-tools && \
  source /opt/ros/humble/setup.bash && \
  echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
  print INFO "Installed ROS2 Humble"

  sudo apt -qq update  && \
  sudo apt -qq -y install python3 python3-pip python3-colcon-common-extensions && \
  echo 'source /home/pib/ros_working_dir/install/setup.bash' >> "$HOME/.bashrc" && \
  echo "export ROS_LOCALHOST_ONLY=1" >> "$HOME/.bashrc"
  print INFO "Installed colcon"


  sudo apt -qq -y install ros-humble-rosbridge-server
  # Install driver for webots connection
  sudo apt -qq -y install ros-humble-webots-ros2-driver

  print INFO "Installed rosbridge-server and Webots driver"
  print INFO "Finished installing ROS2 Humble"
}


# Install tinkeforge daemon and viewer for motors
function install_tinkerforge() {
   if command_exists brickd; then
        print WARN "brick daemon already installed; skipping installation"
   else
    sudo apt -qq update && \
    sudo apt -qq install libusb-1.0-0 libudev1 procps

    # Install brickd
    PLATFORM_TYPE=$(uname -m)
    if [ "$PLATFORM_TYPE" != 'aarch64' ]; then
      echo "Installing Brick daemon for $PLATFORM_TYPE"
      curl --location https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_amd64.deb --output  "$APP_DIR/brickd_linux_latest_amd64.deb"
      sudo dpkg -i "$APP_DIR/brickd_linux_latest_amd64.deb" || { print ERROR "Failed install brickd"; return 1; }
    else
      echo "Installing Brick daemon for $PLATFORM_TYPE"
      curl --location https://download.tinkerforge.com/tools/brickd/linux/brickd_linux_latest_arm64.deb --output "$APP_DIR/brickd_linux_latest_arm64.deb"
      sudo dpkg -i "$APP_DIR/brickd_linux_latest_arm64.deb" || { print ERROR "Failed install brickd"; return 1; }
    fi
    print INFO "Installed brick daemon"
  fi

  print INFO "Setting up pib motors"


   if command_exists brickv; then
        print WARN "brick viewer already installed; skipping installation"
   else
    # Install brickv
    sudo apt -qq install -y python3-pyqt5 python3-pyqt5.qtopengl python3-serial python3-tz python3-tzlocal
    curl --location https://download.tinkerforge.com/tools/brickv/linux/brickv_linux_latest.deb --output "$APP_DIR/brickv_linux_latest.deb"
    sudo dpkg -i "$APP_DIR/brickv_linux_latest.deb" || { print ERROR "Failed install brickv"; return 1; }
    # Tinkerforge python APIs
    curl --silent --location https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg | sudo tee /etc/apt/trusted.gpg.d/tinkerforge.gpg > /dev/null
    echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/tinkerforge.list
    sudo apt -qq update && \
    sudo apt -qq install -y python3-tinkerforge
    print INFO "Installed brick viewer"
  fi
  print SUCCESS "Finished setting up pib motors"
}


# Install Flask API via pipenv
function install_flask_api() {
  print INFO "Install pib-api"

  echo "export PYTHONIOENCODING=utf-8" >> "$HOME/.bashrc"
  pip install pipenv
  print INFO "Installed pipenv"
  cp -r "$PIB_API_SETUP_DIR/flask" "$PIB_API_DIR"
  sudo mv "$PIB_API_DIR/pib_api_boot.service" /etc/systemd/system || print WARN "pib_api_boot.service not found"

  # service enabled at the end of the script
  sudo chmod 777 "$HOME"
  sudo chmod 777 "$PIB_API_DIR"

  print INFO "Finished installing pib-api"
}


# Install ros_packages (voice_assistant, camera, motors, programs, etc.)
function install_ros_packages() {
  print INFO "Installing ros_packages"

  # Camera Dependencies
  sudo curl --silent --location https://docs.luxonis.com/install_dependencies.sh | sudo bash
  python3 -m pip install depthai
  git clone --recurse-submodules https://github.com/luxonis/depthai-python.git
  cd depthai-python/examples || { print ERROR "depthai-python/examples not found"; return 1; }
  python3 install_requirements.py
  # Hand tracker
  git clone https://github.com/geaxgx/depthai_hand_tracker.git
  cd depthai_hand_tracker || { print ERROR "depthai_hand_tracker not found"; return 1; }
  pip install -r requirements.txt
  cd "$HOME" || { print ERROR "${HOME} not found"; return 1; }

  # SLAM dependencies (optional)
#  sudo apt install -y ros-humble-depthai-ros
#  sudo apt install -y ros-humble-rtabmap
#  sudo apt install -y ros-humble-rtabmap-launch
#  sudo apt install -y ros-humble-rtabmap-examples
  print INFO "Installed camera dependencies"


  # VoiceAssistant dependencies
  sudo apt-get install -y portaudio19-dev
  sudo apt-get install flac
  pip install -r "$BACKEND_DIR/ros_packages/voice_assistant/requirements.txt"
  pip install "$BACKEND_DIR/public_api_client"
  pip install "$PIB_API_SETUP_DIR/client"
  pip install pyaudio
  mkdir "$HOME/public_api"
  printf "{\n\t\"trybUrlPrefix\": \"\",\n\t\"publicApiToken\": \"\"\n}\n" > "$HOME/public_api/config.json"
  print INFO "Installed voice assistant dependencies"

  # Programs dependencies
  sudo apt-get install -y python3.10-venv
  readonly USER_PROGRAM_ENV_DIR="$ROS_WORKING_DIR/src/programs/user_program_env"
  mkdir "$USER_PROGRAM_ENV_DIR"
  sudo chmod 700 "$USER_PROGRAM_ENV_DIR"
  python3 -m venv "$USER_PROGRAM_ENV_DIR"
  source "$USER_PROGRAM_ENV_DIR/bin/activate"
  python3 -m pip install numpy==1.26.3
  python3 -m pip install depthai==2.24.0.0
  python3 -m pip install blobconverter==1.4.2
  deactivate
  print INFO "Installed programs dependencies"

  # Motor dependencies
  sudo apt-get -y install libusb-1.0-0-dev
  pip install -r "$BACKEND_DIR/ros_packages/motors/requirements.txt"
  pip install "$BACKEND_DIR/ros_packages/motors/pib_motors"
  print INFO "Installed motors dependencies"

  # move ros-packages into working directory
  cp -r "$BACKEND_DIR/ros_packages/." "$ROS_WORKING_DIR/src"
  sudo chmod -R 700 "$ROS_WORKING_DIR"


  print INFO "Setting up boot-services"
  # Boot camera
  sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.sh"
  sudo chmod 700 "$ROS_CAMERA_BOOT_DIR/ros_camera_boot.service"
  sudo cp "${ROS_CAMERA_BOOT_DIR}/ros_camera_boot.service" /etc/systemd/system


  # Boot motor nodes
  sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.sh"
  sudo chmod 700 "$ROS_MOTORS_BOOT_DIR/ros_motor_boot.service"
  sudo cp "${ROS_MOTORS_BOOT_DIR}/ros_motor_boot.service" /etc/systemd/system


  # Boot voice-assistant
  sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.sh"
  sudo chmod 700 "$ROS_VOICE_ASSISTANT_BOOT_DIR/ros_voice_assistant_boot.service"
  sudo cp "${ROS_VOICE_ASSISTANT_BOOT_DIR}/ros_voice_assistant_boot.service" /etc/systemd/system


  # Boot program node
  sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.sh"
  sudo chmod 700 "$ROS_PROGRAMS_BOOT_DIR/ros_program_boot.service"
  sudo cp "${ROS_PROGRAMS_BOOT_DIR}/ros_program_boot.service" /etc/systemd/system

  cd "$ROS_WORKING_DIR" || { print ERROR "${ROS_WORKING_DIR} not found"; return 1; }
  source /opt/ros/humble/setup.bash
  colcon build || { print ERROR "could not colcon build packages"; return 1; }
  cd "$HOME" || { print ERROR "${HOME} not found"; return 1; }

  echo "SETUP FILES: ${SETUP_FILES} ROS_WORKING_DIR: ${ROS_WORKING_DIR}"
  echo "$(ls $SETUP_FILES)"
  cp "${SETUP_FILES}/ros_config.sh" "$ROS_WORKING_DIR" || { print ERROR "could not move ros_config.sh"; return 1; }
  # services enabled at the end of the script

  print SUCCESS "Finished installing ros_packages"
}


# Install Cerebra, phpLiteAdmin, nginx
function install_frontend() {
  print INFO "Install Cerebra"
  sudo apt -qq install -y nginx
  sudo mkdir -p $DEFAULT_NGINX_HTML_DIR
  # Setting up nginx to serve Cerebra locally
  sudo cp "${SETUP_FILES}/nginx.conf" "$DEFAULT_NGINX_DIR/nginx.conf"

  # Remove pre-installed node version in preparation of node install via nvm
  sudo apt-get purge -y nodejs
  # Install Node Version Manager. Version is hardcoded to avoid discrepancies through updates
  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash

  # make nvm available without re-opening terminal
  export NVM_DIR="$HOME/.nvm"
  [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" # This loads nvm
  [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" # This loads nvm bash_completion

  if ! command_exists nvm; then
      print ERROR "nvm installation failed"
      return 1
  fi


  # Install and use Node.js 18 via nvm
  # Dont use sudo for nvm-associated commands (npm, ng) since nvm is not accessible by root
  nvm install 18
  nvm use 18

  # Install Angular CLI
  npm install -g @angular/cli
  npm link @angular/cli

  npm --prefix "$FRONTEND_DIR" install
  cd "$FRONTEND_DIR" || { print ERROR "${FRONTEND_DIR} not found"; return 1; }
  ng build --configuration production
  cd "$HOME" || { print ERROR "${HOME} not found"; return 1; }

  # Move the build to the destination folder
  sudo mv "$FRONTEND_DIR/dist"/* "$DEFAULT_NGINX_HTML_DIR"

  cp "${SETUP_FILES}/ros_cerebra_boot.sh" "${ROS_WORKING_DIR}/ros_cerebra_boot.sh"
  sudo chmod 700 "$ROS_WORKING_DIR"/ros_cerebra_boot.sh

  sudo cp "${SETUP_FILES}/ros_cerebra_boot.service" /etc/systemd/system
  sudo chmod 700 /etc/systemd/system/ros_cerebra_boot.service
  # service enabled at the end of the script

  print INFO "Build frontend and setup nginx"

  # Install and configure phpLiteAdmin
  sudo apt -qq update && sudo apt -qq install -y php8.1-fpm php-sqlite3
  sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
  sudo mkdir "$PHPLITEADMIN_INSTALLATION_DIR" || print WARN "$PHPLITEADMIN_INSTALLATION_DIR already exists"
  sudo chown -R www-data:www-data "$PHPLITEADMIN_INSTALLATION_DIR"
  sudo chmod -R 700 "$PHPLITEADMIN_INSTALLATION_DIR"
  sudo unzip -o "$SETUP_FILES/$PHPLITEADMIN_ZIP" -d "$PHPLITEADMIN_INSTALLATION_DIR" || print ERROR "cannot unzip $PHPLITEADMIN_ZIP"
  sudo systemctl restart php8.1-fpm || { print ERROR "cannot start phpliteadmin"; return 1; }
  print INFO "Installed phpLiteAdmin"


  print SUCCESS "Finished installing cerebra"
}


function install_blocky_node_service() {
  print INFO "Install Blockly Node service"

  # npm needs to be sourced, happens in install_frontend
  if ! command_exists npm; then
        print ERROR "npm not found"
        return 1
  fi


  # copy the pib-blockly-server and client from the setup-folder to their targets in the user home-dir
  cp -r "$PIB_BLOCKLY_SETUP_DIR/pib_blockly_server" "$HOME"
  cp -r "$PIB_BLOCKLY_SETUP_DIR/pib_blockly_client" "$HOME"

  # build the pib-blockly-server
  cd "$PIB_BLOCKLY_SERVER_DIR" || { print ERROR "${PIB_BLOCKLY_SERVER_DIR} not found"; return 1; }
  npm install
  npm run build

  # build the pib-blockly-server
  cd "$PIB_BLOCKLY_SERVER_DIR" || { print ERROR "${PIB_BLOCKLY_SERVER_DIR} not found"; return 1; }
  npm install || { print ERROR "error using npm"; return 1; }
  npm run build || { print ERROR "error using npm"; return 1; }

  # create service that starts the pib_blockly_server during boot
  sudo cp "${PIB_BLOCKLY_SERVER_DIR}/pib_blockly_server_boot.service" /etc/systemd/system
  # services enabled at end of the script

  # install the pib_blockly_client
  pip install "$PIB_BLOCKLY_SETUP_DIR/pib_blockly_client"

  cp "${SETUP_FILES}/start_json_server.sh" "$HOME"
  sudo chmod 700 "${HOME}"/start_json_server.sh
  print SUCCESS "Finished installing Blockly Node service"
}



if ! [ "$DIST_VERSION" == "jammy" ]; then
    print ERROR "Not using Ubuntu 22.04; cannot install locally"
    return 1
fi

if ! [ "$USER" == "pib" ]; then
  print ERROR "User $USER detected; expected 'pib'; cannot install locally"
  return 1
fi


# Disable IPv6 due to installation problems with npm otherwise (not necessary for Docker setup)
print INFO "Disable IPv6"
sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1

install_ros || { print ERROR "Failed installing ROS"; return 1; }
install_tinkerforge || { print ERROR "Failed installing Tinkerforge"; return 1; }
install_flask_api || { print ERROR "Failed installing pib-api"; return 1; }
install_ros_packages || { print ERROR "Failed installing ros_packages"; return 1; }
install_frontend || { print ERROR "Failed installing frontend"; return 1; }
install_blocky_node_service || { print ERROR "Failed installing blocky node service"; return 1; }

# Enable all services
sudo systemctl daemon-reload
sudo systemctl enable pib_blockly_server_boot.service --now
sudo systemctl enable pib_api_boot.service --now
sudo systemctl enable ros_camera_boot.service --now
sudo systemctl enable ros_motor_boot.service --now
sudo systemctl enable ros_program_boot.service --now
sudo systemctl enable ros_voice_assistant_boot.service --now
sudo systemctl enable ros_cerebra_boot.service --now