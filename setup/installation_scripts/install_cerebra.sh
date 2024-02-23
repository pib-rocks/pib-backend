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
DATABASE_DIR="$USER_HOME/pib_data"
DATABASE_FILE="pibdata.db"

# pib-api variables
PIB_API_DIR="$USER_HOME/flask"

# python code variables
PYTHON_CODE_PATH="$USER_HOME/cerebra_programs"
INIT_PYTHON_CODE="print('hello world')"

# Setup nginx
sudo apt install -y nginx
sudo mkdir -p $DEFAULT_NGINX_HTML_DIR

# Setting up nginx to serve Cerebra locally
sudo cp "$SETUP_FILES/nginx.conf" "$DEFAULT_NGINX_DIR/$NGINX_CONF_FILE"

# Install NVM (Node Version Manager)
NVM_DIR="/etc/nvm"
sudo mkdir "$NVM_DIR"
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/master/install.sh | bash

# Source NVM script to make it available in the current shell
source ~/.nvm/nvm.sh

# Install version 18 of Node.js and use it
sudo nvm install 18
sudo nvm use 18

# Install Angular CLI globally
sudo npm cache clean -f
sudo npm install -g npm@latest
sudo npm install -g @angular/cli --legacy-peer-deps

# Install Cerebra project dependencies
sudo npm --prefix "$FRONTEND_DIR" install

# Build the Angular app without changing directory
sudo ng build --configuration productive --project "$FRONTEND_DIR"

# Move the build to the destination folder
sudo mv "$FRONTEND_DIR/dist"/* "$DEFAULT_NGINX_HTML_DIR"

# Install and configure phpLiteAdmin
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
sudo mkdir "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chown -R www-data:www-data "$PHPLITEADMIN_INSTALLATION_DIR"
sudo chmod -R 755 "$PHPLITEADMIN_INSTALLATION_DIR"
sudo unzip "$SETUP_FILES/$PHPLITEADMIN_ZIP" -d "$PHPLITEADMIN_INSTALLATION_DIR"
sudo systemctl restart php8.1-fpm

# Create the database (if it doesn't exist) and initialize it with the SQL file
mkdir "$DATABASE_DIR"
sudo chmod 777 "$USER_HOME"
sudo chmod 777 "$DATABASE_DIR"
sqlite3 "$DATABASE_DIR/$DATABASE_FILE" < "$SETUP_FILES/cerebra_init_database.sql"
sudo chmod 766 "$DATABASE_DIR/$DATABASE_FILE"
echo -e "$NEW_LINE""Database initialized successfully!"

# Create the directory for python code and populate it with a single initial python script (matching
# the single entry in the database)
mkdir "$PYTHON_CODE_PATH"
echo "$INIT_PYTHON_CODE" | cat > "$PYTHON_CODE_PATH/e1d46e2a-935e-4e2b-b2f9-0856af4257c5.py"

# Create pib-api
echo "export PYTHONIOENCODING=utf-8" >> $USER_HOME/.bashrc
pip3 install pipenv
cd $USER_HOME

pip install "$PIB_API_SETUP_DIR/client"
cp -r "$PIB_API_SETUP_DIR/flask" "$PIB_API_DIR"
sudo mv "$PIB_API_DIR/pib_api_boot.service" /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pib_api_boot.service
cd $USER_HOME

# Open firefox without gui to generate default folder structures 
# This also avoids the welcome page the first time a user opens the browser
timeout 20s firefox --headless

# Set localhost as homepage in default profile
readonly FIREFOX_PREFS_FILE=$(echo /home/pib/snap/firefox/common/.mozilla/firefox/*.default)/prefs.js
echo "user_pref(\"browser.startup.homepage\", \"127.0.0.1\");" >> "$FIREFOX_PREFS_FILE"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Cerebra installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"