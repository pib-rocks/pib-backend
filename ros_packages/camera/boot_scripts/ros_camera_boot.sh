#!/bin/bash
source /home/pib/pib-venv/bin/activate
source /opt/ros/jazzy/setup.bash
source /home/pib/ros_working_dir/install/setup.bash
source /home/pib/ros_working_dir/ros_config.sh
ros2 run oak_d_lite stereo 2>&1 | tee -a ~/ros_working_dir/src/camera/stereo.log