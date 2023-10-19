# Software setup

This script assumes: 
- that Ubuntu Desktop 22.04.2 LTS is installed
- the user running it is **pib**

If you have not set up the user **pib** at installation, you can do so via the settings-dialog of Ubuntu and then log in as **pib**.

## Running the script

To run the script do the next steps:

1. Open a terminal

2. Download the script with the following command:

        wget https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup-pib.sh

3. Make the script executable:
   
        chmod 755 ./setup-pib.sh

4. Let the script run:

        ./setup-pib.sh

The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...
Once the installation is complete, please restart the system to apply all the changes.

## Checking if the software started successfully

To check if ROS2 successfully started:

        systemctl status ros_cerebra_boot.service

To check if Cerebra successfully started:

        sudo systemctl status nginx

To check if the camera node successfully started:

        systemctl status ros_camera_boot.service

With the following command, you can check the running ros2 nodes:

        ros2 node list
		
## Clustering pibs

To enable clustering of pibs on default ROS_DOMAIN_ID=0:

1. Open a Terminal:
2. Run the following command:
		gedit ~/.bashrc 
	OR for users connected through terminal:
		vim ~/.bashrc 
3. Within .bashrc delete:
		export ROS_LOCALHOST_ONLY=1 	
	or 
		set to 0
4. Restart pib

To add pib to a distinct logical network:

1. Open a Terminal 
2. Run the following command:
		gedit ~/.bashrc 
	OR for users connected through terminal:
		vim ~/.bashrc
3. Delete:
		export ROS_LOCALHOST_ONLY=1
4. Append:
		export ROS_DOMAIN_ID=YOUR_DOMAIN_ID
5. Restart pib

For a range of available ROS_DOMAIN_IDs please check the official documentation at
	https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html

# Software update
This script assumes: 
- that Ubuntu Desktop 22.04.2 LTS is installed
- the user running it is **pib**
- the setup script was executed successfully

1. Open a terminal

2. Run the update script:
        
        update-pib -Cerebra
   
   or

        update-pib
   
You can use "update-pib -Cerebra" to update only the Website Cerebra.
If you execute the command without the parameter "-Cerebra" the website is updated and all packages.
