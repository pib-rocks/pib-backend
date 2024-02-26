source /opt/ros/humble/setup.bash;
echo "COLCON BUILD"; 
colcon build; 
echo "running install/setup.bash"
source install/setup.bash;
echo "starting ROS services";
ros2 launch rosbridge_server rosbridge_websocket_launch.xml & >> log.output
ros2 run voice_assistant assistant >> log.output &
ros2 run programs program >> log.output &
ros2 run programs proxy_program >> log.output &
ros2 run oak_d_lite stereo >> log.output &
ros2 run motors motor_control >> log.output &
ros2 run motors motor_current >> log.output &
tail -f log.output