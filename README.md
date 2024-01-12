# Software setup

This script assumes: 
- that Ubuntu Desktop 22.04.2 LTS is installed
- the user running it is **pib**

If you have not set up the user **pib** at installation, you can do so via the settings-dialog of Ubuntu and then log in as **pib**.

## Running the script

To run the script do the next steps:

1. Open a terminal in Ubuntu

2. Insert the following command into the terminal to download the script:

        wget https://raw.githubusercontent.com/pib-rocks/setup-pib/main/setup-pib.sh

	(or download it manually: https://github.com/pib-rocks/setup-pib/blob/main/setup-pib.sh)

3. Insert this into the terminal to make the script executable:
   
        chmod 755 ./setup-pib.sh

4. Insert this command to run the script:

        ./setup-pib.sh

The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...
Once the installation is complete, please restart the system to apply all the changes.

# Updating the Software
This script assumes that the setup script was executed successfully

1. Open a terminal
2. Run the update script: `update-pib`  or  `update-pib -Cerebra`
   
You can use the "-Cerebra" parameter to update only the Cerebra browser application.
If you execute the command without the parameter "-Cerebra", the browser app and all packages are updated.

## Checking if the software was installed successfully

Inside the "dev_tools" folder of our Github repo, you can find a shell script (health-check-pib.sh) that checks if 
all necessary packages are installed and all required ros-services are running.

Follow these steps to run the health-check-script:
1. Download the script from our Github repo: https://github.com/pib-rocks/setup-pib/blob/main/dev_tools/health-check-pib.sh 
2. Change the permissions of the file "chmod 755 health-check-pib.sh"
3. Run the script "./health-check-pib.sh"

The script also has a development mode "./health-check-pib.sh -d" option that allows you to only check some part of the script.
For example to only check ros packages and services without the python packages.

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