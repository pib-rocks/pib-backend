# Software setup

This script assumes:

- that the newest Raspberry Pi OS is installed
- the user running it is **pib**

## Installing pibs software

All the software pib requires can be installed by running our setup script.
Follow these steps to run it:

1. Open a terminal in Raspberry Pi OS

2. Insert the following command into the terminal to download the script:

        wget https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup-pib.sh

   (or download it manually: https://github.com/pib-rocks/pib-backend/blob/main/setup/setup-pib.sh)

3. Insert this command to run the script:

        bash setup-pib.sh

   If you want to run the setup-script in legacy mode (for Raspberry Pi 4), insert:
               
         bash setup-pib.sh -l

The setup then adds Cerebra and it's dependencies, including ROS2, Tinkerforge,...
Once the installation is complete, please restart the system to apply all the changes.

# Updating the Software

This script assumes that the setup script was executed successfully

1. Open a terminal
2. Enter this command: `update-pib`

This script will update your docker containers (Front- and Backend)

## Webots

Starting the webots simulation:

1. Complete all steps of the "Installing pibs software"-section of this readme document
2. Enter the following command into a terminal:  
   `ros2 launch pibsim_webots pib_launch.py`  
   (The first time this command is entered, a prompt will appear asking to install webots.  
   Confirm this prompt and wait a few seconds for the installation to finish. Webots should open afterwards.)

Webots may throw error messages saying it crashed (especially on VM). This can usually be ignored by clicking on "wait".

## Clustering pibs

To synchronize communication between pibs on default ROS_DOMAIN_ID=0:

1. Open a Terminal:
2. Run the following command:  
   `gedit ~/.bashrc`  
   OR for users connected through terminal:  
   `vim ~/.bashrc`
3. Within .bashrc  
   delete: export ROS_LOCALHOST_ONLY=1  
   or replace it with: ROS_LOCALHOST_ONLY=0
4. Restart pib

To add pib to a distinct logical network:

1. Open a Terminal
2. Run the following command:  
   `gedit ~/.bashrc`  
   OR for users connected through terminal:  
   `vim ~/.bashrc`
3. Delete: "export ROS_LOCALHOST_ONLY=1"
4. Append: "export ROS_DOMAIN_ID=YOUR_DOMAIN_ID"
5. Restart pib

For a range of available ROS_DOMAIN_IDs please check the official documentation at:  
https://docs.ros.org/en/dashing/Concepts/About-Domain-ID.html

## Update Servo Bricklet IDs

1. Open a terminal and navigate to `pib-backend/scripts`
2. For Servo Bricklet ID update you can run a script with `python3 update-servo-ids.py`
after that you can change every single uid in a command line. 
3. You can change the IDs for Bricklets 1, 2 and 3 (to skip a Bricklet simply press enter without typing anything).

**Attention**: The IDs are case-sensitive and consist only of letters and numbers


Example: 
   Standard bricklet uids: 
```
1  AAA
2  BBB
3  CCC
```

execute `python3 update-servo-ids.py` and change one after another all bricklet uids:
```
1  AAA -> 1 Aa1
2  BBB -> 2 Bb2
3  CCC -> 3 Cc3
```

### Docker

The backend can be started via `docker compose`. Since the software requires to interface with the OS hardware (USB,
sound and GPIO) Docker for Windows and Mac is not supported.
Running `docker compose up` will start the Flask API, rosbridge and the blockly node server. To run the full backend,
including camera, motors, programs and the voice assistant, profiles can be used:

```bash
docker compose --profile all up
```

`password.env` required to run the voice assistant:

```
TRYB_URL_PREFIX=<BASE_URL_Tryb>
```
