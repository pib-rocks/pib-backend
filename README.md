# Software setup

This script assumes: 
- that Ubuntu Desktop 22.04.2 LTS is installed
- the default-user executing it is set to "**pib**". If your user is different, change the DEFAULT_USER variable in the script accordingly.
The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...

To run the script do the next steps:

1. Install *curl* if you don't have it:

        sudo apt install curl

2. Download the script with *curl*:

        curl "https://raw.githubusercontent.com/pib-rocks/setup-pib/main/ros_and_cerebra_setup.sh" -O

3. Make the script executable:
   
        chmod 755 ./ros_and_cerebra_setup.sh

4. Let the script run:

        ./ros_and_cerebra_setup.sh

Once the installation is complete, the system will automatically reboot.

## Checking if the software started successfully

To check if ROS2 successfully started:

    systemctl status ros_cerebra_boot.service

To check if Cerebra successfully started:

    sudo systemctl status nginx

With the following command, you can check the running ros2 nodes:

    ros2 node list