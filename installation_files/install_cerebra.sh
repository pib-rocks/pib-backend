#!/bin/bash
#
# This script installs Cerebra

echo -e "$YELLOW_TEXT_COLOR""-- Installing Cerebra --""$RESET_TEXT_COLOR"		

# Nginx variables
DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
NGINX_CONF_FILE="nginx.conf"
NGINX_CONF_FILE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/nginx.conf"

# Cerebra download location
export CEREBRA_ARCHIVE_URL_PATH="https://pib.rocks/wp-content/uploads/pib_data/cerebra-latest.zip"
export CEREBRA_ARCHIVE_NAME="cerebra-latest.zip"

# Database variables
PHPLITEADMIN_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"
DATABASE_DIR="$USER_HOME/pib_data"
DATABASE_FILE="pibdata.db"
DATABASE_INIT_QUERY_FILE="cerebra_init_database.sql"
DATABASE_INIT_QUERY_LINK="https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup_files/cerebra_init_database.sql"

# pib api variables
PIB_API_DIR="$USER_HOME/flask"
PIB_API_URL_PATH="https://github.com/pib-rocks/pib-api/archive/refs/heads/main.zip"

echo -e '\nInstall nginx...'
sudo apt install -y nginx
# If the 'html' directory inside of nginx doesn't exist, it will be created
if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then sudo -S mkdir -p $DEFAULT_NGINX_HTML_DIR; fi
echo -e '\nClean up the html directory...'
cd $DEFAULT_NGINX_HTML_DIR && sudo -S rm -r *

# Create temporary folder for cerebra download
cd $USER_HOME
mkdir "$TEMPORARY_SETUP_DIR"
# Download Cerebra artifact to the working directory
echo -e '\nDownloading Cerebra application'
curl $CEREBRA_ARCHIVE_URL_PATH -L --output "$TEMPORARY_SETUP_DIR/$CEREBRA_ARCHIVE_NAME"
#
# Unzip cerebra files to nginx
echo -e '\nUnzip cerebra...'
#cd $RASP_TMP_FOLDER
sudo unzip "$TEMPORARY_SETUP_DIR/$CEREBRA_ARCHIVE_NAME" -d $DEFAULT_NGINX_HTML_DIR
#
# Setting up nginx to serve Cerebra locally
echo -e '\nDownloading nginx configuration file...'
sudo curl $NGINX_CONF_FILE_URL --output $DEFAULT_NGINX_DIR/$NGINX_CONF_FILE

#
# Install and configure phpLiteAdmin #TODO: Use temporary setup folder instead of ros_ws
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
curl $PHPLITEADMIN_LINK -L --output $ROS_WORKING_DIR/$PHPLITEADMIN_ZIP
sudo mkdir $PHPLITEADMIN_INSTALLATION_DIR
sudo chown -R www-data:www-data $PHPLITEADMIN_INSTALLATION_DIR
sudo chmod -R 755 $PHPLITEADMIN_INSTALLATION_DIR
sudo unzip $ROS_WORKING_DIR/$PHPLITEADMIN_ZIP -d $PHPLITEADMIN_INSTALLATION_DIR
sudo systemctl restart php8.1-fpm

# Create the database (if it doesn't exist) and initialize it with the SQL file #TODO: Use temporary setup folder instead of ros_ws
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

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Cerebra installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"