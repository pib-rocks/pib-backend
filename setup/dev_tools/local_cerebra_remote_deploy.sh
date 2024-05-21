#!/bin/bash
#
# This script builds an executable version of your local cerebra repo
# and moves it to the specified ip (raspberry pi or vm).

# Instructions:
# 1. Change the path and ip variables as mentioned below
# 2. Run this script in a windows bash terminal.

# Change "LOCAL_CEREBRA_DIR" to your local main cerebra folder
# Example: LOCAL_CEREBRA_DIR="C:\Users\Example User\Documents\cerebra2"
LOCAL_CEREBRA_DIR="C:\Users\User Name\Documents\cerebra2"
LOCAL_BUILD_DIR="/dist/cerebra"

# Change "TARGET_IP" to the desired target raspi or virtal machine
# Examples: "192.168.220.109" or "localhost" for VM-use
TARGET_IP="localhost"
TARGET_USERNAME="pib"

TEMPORARY_DIR="/home/${TARGET_USERNAME}/cerebra"
DEFAULT_NGINX_HTML_DIR="/etc/nginx/html"

# Resolve possible ssh fingerpint/ key issues
ssh-keygen -R "$TARGET_IP"

### deploy local cerebra via remote ssh connection
# Change directory to cerebra project folder
echo "Starting remote deployment of your local cerebra version."
cd "$LOCAL_CEREBRA_DIR"

# Overwrite previous build
echo "Rebuilding cerebra..."
ng build --configuration production

# Check if ng build was successfull
EXIT_CODE=$?
if [ "$EXIT_CODE" -ne "0" ] 
then
    echo "Some error occured while trying to ng build"
    echo "Check if you're directory is referenced correctly"
    exit 1
fi

# Stop nginx
echo 'Trying to stop nginx...'
ssh $TARGET_USERNAME@$TARGET_IP "sudo -S systemctl stop nginx"

# If the project directory inside of nginx doesn't exist, it will be created
echo 'Checking if directory nginx exists...'
ssh $TARGET_USERNAME@$TARGET_IP "if [ ! -d $DEFAULT_NGINX_HTML_DIR ]; then sudo -S mkdir -p $DEFAULT_NGINX_HTML_DIR; fi"

# Clear project directory inside of nginx
echo 'Deleting previous cereba version...'
ssh $TARGET_USERNAME@$TARGET_IP "cd "$DEFAULT_NGINX_HTML_DIR" && sudo -S rm -r * "

# Copy app files from local machine to the tempopary folder in Raspberry
echo 'Copiyng files to Raspberry...'
scp -r "${LOCAL_CEREBRA_DIR}""${LOCAL_BUILD_DIR}" $TARGET_USERNAME@$TARGET_IP:$TEMPORARY_DIR

# Copy app files from the tempopary folder to the nginx directory
ssh $TARGET_USERNAME@$TARGET_IP "sudo -S mv $TEMPORARY_DIR $DEFAULT_NGINX_HTML_DIR/cerebra"
# Start nginx anew
echo 'Trying to restart nginx...'
ssh $TARGET_USERNAME@$TARGET_IP "sudo -S systemctl restart nginx"

echo 'Cerebra remote redeployment done!'