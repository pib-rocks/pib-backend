#!/bin/bash
#
# This script installs Cerebra
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing Cerebra --""$RESET_TEXT_COLOR"		

# Nginx variables
DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
NGINX_CONF_FILE="nginx.conf"

# Database variables
PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"

# Pib-api variables
PIB_API_DIR="$USER_HOME/flask"
DATABASE_FILE="pibdata.db"

# Setup nginx
sudo apt install -y nginx
sudo mkdir -p $DEFAULT_NGINX_HTML_DIR

# Setting up nginx to serve Cerebra locally
sudo cp "$SETUP_FILES/nginx.conf" "$DEFAULT_NGINX_DIR/$NGINX_CONF_FILE"

# Remove pre-installed node version in preparation of node install via nvm
sudo apt-get purge -y nodejs

# Install Node Version Manager. Version is hardcoded to avoid discrepancies through updates
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash

# Move nvm directory to systemfolders and create environment variable
NVM_DIR="/etc/nvm"
sudo mv "$USER_HOME/.nvm/" "$NVM_DIR"
export NVM_DIR="$NVM_DIR"

# Load nvm into bash
source "$NVM_DIR/nvm.sh"

# Install and use Node.js 18 via nvm
# Dont use sudo for nvm-associated commands (npm, ng) since nvm is not accessible by root
nvm install 18
nvm use 18

# Install Angular CLI
npm install -g @angular/cli
npm link @angular/cli

# Install app dependencies and build app
npm --prefix "$FRONTEND_DIR" install
cd "$FRONTEND_DIR"
ng build --configuration production
cd "$USER_HOME"

# Move the build to the destination folder
sudo mv "$FRONTEND_DIR/dist"/* "$DEFAULT_NGINX_HTML_DIR"

# Install and configure phpLiteAdmin
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
sudo mkdir "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chown -R www-data:www-data "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chmod -R 700 "$PHPLITEADMIN_INSTALLATION_DIR"
sudo unzip "$SETUP_FILES/$PHPLITEADMIN_ZIP" -d "$PHPLITEADMIN_INSTALLATION_DIR"
sudo systemctl restart php8.1-fpm

# Create pib-api
echo "export PYTHONIOENCODING=utf-8" >> "$USER_HOME/.bashrc"
pip3 install pipenv
cd "$USER_HOME"

pip install "$PIB_API_SETUP_DIR/client"
cp -r "$PIB_API_SETUP_DIR/flask" "$PIB_API_DIR"
sudo mv "$PIB_API_DIR/pib_api_boot.service" /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pib_api_boot.service
cd "$USER_HOME"
sudo chmod 777 "$USER_HOME"
sudo chmod 777 "$PIB_API_DIR"
sqlite3 "$PIB_API_DIR/$DATABASE_FILE" "VACUUM;"
sudo chmod 766 "$PIB_API_DIR/$DATABASE_FILE"


# Set 
# Open firefox without gui to generate default folder structures 
# This also avoids the welcome page the first time a user opens the browser
timeout 20s firefox --headless

# Set localhost as homepage in default profile
readonly FIREFOX_PREFS_FILE=$(echo /home/pib/snap/firefox/common/.mozilla/firefox/*.default)/prefs.js
echo "user_pref(\"browser.startup.homepage\", \"127.0.0.1\");" >> "$FIREFOX_PREFS_FILE"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Cerebra installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"