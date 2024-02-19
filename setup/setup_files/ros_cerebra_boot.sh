#!/bin/bash
source /home/pib/ros_working_dir/ros_config.sh
sudo service nginx restart
source /opt/ros/humble/setup.bash
source /home/pib/ros_working_dir/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
