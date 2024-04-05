source /opt/ros/humble/setup.bash;
echo "COLCON BUILD"; 
colcon build; 
echo "running install/setup.bash"
source install/setup.bash;
echo "starting ROS services";
ros2 launch pib_launch.yaml