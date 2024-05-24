# Load nvm into bash

source ~/.bashrc

cd /home/pib/ros_working_dir/json-server/

npm install
npm audit fix

# Stop if still running pib_api service
systemctl stop pib_api_boot.service

# Start the JSON-Server
node server/json-server-funktion.mjs
