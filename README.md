# Software setup

This script assumes: 
- that Ubuntu Desktop 22.04.2 LTS is installed
- the default-user executing it is set to **pib**.

## Creating the user
Create new user doing the following steps:

1. Create user named **pib** with *adduser* command:

        adduser pib

2. You will be prompted to set and confirm the password for new user (you can also set it to "**pib**" for convenience).
3. Add the new user to the *sudo* group:

        usermod -aG sudo pib

4. Test the sudo access:

        groups pib

   This prints all the groups that the **pib** user belongs to.

5. Optionally, you can make it so the new user can execute sudo commands without being prompted for a password:  
   Open the *sudoers* file:
      
        sudo visudo

   At the end of the file add the following line:

        pib ALL=(ALL) NOPASSWD: ALL

   Save and close the *sudoers* file.

## Running the script
To run the script do the next steps:

1. Install *curl* if you don't have it:

        sudo apt install curl

2. Download the script with *curl*:

        curl "https://raw.githubusercontent.com/pib-rocks/setup-pib/main/ros_and_cerebra_setup.sh" -O

3. Make the script executable:
   
        chmod 755 ./ros_and_cerebra_setup.sh

4. Let the script run:

        ./ros_and_cerebra_setup.sh

The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...  
Once the installation is complete, the system will automatically reboot.

## Checking if the software started successfully

To check if ROS2 successfully started:

    systemctl status ros_cerebra_boot.service

To check if Cerebra successfully started:

    sudo systemctl status nginx

To check if the camera node successfully started:

    systemctl status ros_camera_boot.service

With the following command, you can check the running ros2 nodes:

    ros2 node list