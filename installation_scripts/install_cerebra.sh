#!/bin/bash
#
# This script installs Cerebra
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing Cerebra --""$RESET_TEXT_COLOR"		

# Nginx variables
DEFAULT_NGINX_DIR="/etc/nginx"
DEFAULT_NGINX_HTML_DIR="$DEFAULT_NGINX_DIR/html"
NGINX_CONF_FILE="nginx.conf"
NGINX_CONF_FILE_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/nginx.conf"

# Cerebra download location
export CEREBRA_ARCHIVE_URL="https://pib.rocks/wp-content/uploads/pib_data/cerebra-latest.zip"
export CEREBRA_ARCHIVE_NAME="cerebra-latest.zip"

# Database variables
PHPLITEADMIN_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_ZIP="phpliteadmin_v1_9_9_dev.zip"
PHPLITEADMIN_INSTALLATION_DIR="/var/www/phpliteadmin"
DATABASE_DIR="$USER_HOME/pib_data"
DATABASE_FILE="pibdata.db"
DATABASE_INIT_QUERY_FILE="cerebra_init_database.sql"
DATABASE_INIT_QUERY_URL="https://raw.githubusercontent.com/pib-rocks/setup-pib/""${repo_map[$SETUP_PIB_ORIGIN]}""/setup_files/cerebra_init_database.sql"

# pib api variables
PIB_API_DIR="$USER_HOME/flask"
PIB_API_URL="https://github.com/pib-rocks/pib-api/archive/refs/heads/""${repo_map[$PIB_API_ORIGIN]}"".zip"

# python code variables
PYTHON_CODE_PATH="$USER_HOME/cerebra_programs"
INIT_PYTHON_CODE="print('hello world')"

echo -e "$NEW_LINE""Install nginx..."
sudo apt install -y nginx
# If the 'html' directory inside of nginx doesn't exist, it will be created
if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then sudo -S mkdir -p $DEFAULT_NGINX_HTML_DIR; fi
echo -e "$NEW_LINE""Clean up the html directory..."
cd $DEFAULT_NGINX_HTML_DIR && sudo -S rm -r *

# Download Cerebra artifact to the working directory
cd $USER_HOME
echo -e "$NEW_LINE""Downloading Cerebra application..."
curl "$CEREBRA_ARCHIVE_URL" -L --output "$TEMPORARY_SETUP_DIR/$CEREBRA_ARCHIVE_NAME"
#
# Unzip cerebra files to nginx
echo -e "$NEW_LINE""Unzip cerebra..."
#cd $RASP_TMP_FOLDER
sudo unzip "$TEMPORARY_SETUP_DIR/$CEREBRA_ARCHIVE_NAME" -d $DEFAULT_NGINX_HTML_DIR
#
# Setting up nginx to serve Cerebra locally
echo -e "$NEW_LINE""Downloading nginx configuration file..."
sudo curl $NGINX_CONF_FILE_URL --output $DEFAULT_NGINX_DIR/$NGINX_CONF_FILE

#
# Install and configure phpLiteAdmin
sudo sed -i "s|;cgi.fix_pathinfo=1|cgi.fix_pathinfo=0|" /etc/php/8.1/fpm/php.ini
curl "$PHPLITEADMIN_URL" -L --output "$TEMPORARY_SETUP_DIR/$PHPLITEADMIN_ZIP"
sudo mkdir $PHPLITEADMIN_INSTALLATION_DIR
sudo chown -R www-data:www-data $PHPLITEADMIN_INSTALLATION_DIR
sudo chmod -R 755 $PHPLITEADMIN_INSTALLATION_DIR
sudo unzip "$TEMPORARY_SETUP_DIR/$PHPLITEADMIN_ZIP" -d $PHPLITEADMIN_INSTALLATION_DIR
sudo systemctl restart php8.1-fpm

# Create the database (if it doesn't exist) and initialize it with the SQL file
curl "$DATABASE_INIT_QUERY_URL" -L --output "$TEMPORARY_SETUP_DIR/$DATABASE_INIT_QUERY_FILE"
echo "Creating (if not exist) and initializing SQLite database $DATABASE_FILE with $TEMPORARY_SETUP_DIR/$DATABASE_INIT_QUERY_FILE..."
mkdir $DATABASE_DIR
sudo chmod 777 $USER_HOME
sudo chmod 777 $DATABASE_DIR
sudo sqlite3 "$DATABASE_DIR/$DATABASE_FILE" < "$TEMPORARY_SETUP_DIR/$DATABASE_INIT_QUERY_FILE"
sudo chmod 766 $DATABASE_DIR/$DATABASE_FILE
echo -e "$NEW_LINE""Database initialized successfully!"

# Create the directory for python code and populate it with a single initial python script (matching
# the single entry in the database)
mkdir "$PYTHON_CODE_PATH"
echo "$INIT_PYTHON_CODE" | cat > "$PYTHON_CODE_PATH/e1d46e2a-935e-4e2b-b2f9-0856af4257c5.py"

# Create pib-api
echo "export PYTHONIOENCODING=utf-8" >> $USER_HOME/.bashrc
pip3 install pipenv
cd $USER_HOME
readonly PIB_API_ARCHIVE_NAME="pib-api-""${repo_map[$PIB_API_ORIGIN]}"
readonly PIB_API_ARCHIVE_PATH="$TEMPORARY_SETUP_DIR""/$PIB_API_ARCHIVE_NAME"".zip"
wget -O "$PIB_API_ARCHIVE_PATH" "$PIB_API_URL"
unzip "$PIB_API_ARCHIVE_PATH" -d "$TEMPORARY_SETUP_DIR"
mv "$TEMPORARY_SETUP_DIR/$PIB_API_ARCHIVE_NAME""/flask/" "$PIB_API_DIR"
sudo mv "$PIB_API_DIR""/pib_api_boot.service" "/etc/systemd/system"
sudo systemctl daemon-reload
sudo systemctl enable pib_api_boot.service
cd $USER_HOME

# Allow editing in all src-directories
sudo chmod -R 777 $ROS_WORKING_DIR
sudo chmod -R 777 $ROS_WORKING_DIR/src

# Create firefox profile and initialize it to generate default folder structure
firefox -CreateProfile pib
timeout 20s firefox --headless

# Set localhost as homepage
readonly FIREFOX_PREFS_FILE=$(echo /home/pib/snap/firefox/common/.mozilla/firefox/*.pib)/prefs.js
echo "user_pref(\"browser.startup.homepage\", \"127.0.0.1\");" >> "$FIREFOX_PREFS_FILE"

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Cerebra installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"