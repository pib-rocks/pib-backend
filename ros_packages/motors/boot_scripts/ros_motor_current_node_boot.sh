#!/bin/bash
source /home/pib/ros_working_dir/ros_config.sh
source /opt/ros/humble/setup.bash
source /home/pib/ros_working_dir/install/setup.bash
ros2 run motors motor_current 2>&1 | tee -a ~/ros_working_dir/src/motors/motor_current.log