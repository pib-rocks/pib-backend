# Software setup

The setup script supports two environments:

- **Raspberry Pi OS** (bookworm or trixie): installs and runs Cerebra via **Docker** (production).
- **Ubuntu 24.04 (Noble)**: installs and runs Cerebra **natively** (development: ROS2 Jazzy, no Docker, all components via systemd).

The script assumes the user running it is **pib**.

## Installing pibs software

Run the setup script on either Raspberry Pi OS or Ubuntu 24.04.

1. Open a terminal.

2. Download the script:

        wget https://raw.githubusercontent.com/pib-rocks/pib-backend/main/setup/setup-pib.sh

   (or download it manually: https://github.com/pib-rocks/pib-backend/blob/main/setup/setup-pib.sh)

3. Run the script:

        bash setup-pib.sh

   - On **Raspberry Pi OS** this installs Cerebra via Docker (containers for backend and frontend).
   - On **Ubuntu 24.04** this installs Cerebra natively (ROS2 Jazzy, Flask, Blockly server, ROS nodes, Cerebra frontend; no Docker).

The setup adds Cerebra and its dependencies (including ROS2, Tinkerforge, etc.).
Once the installation is complete, restart the system to apply all changes.

# Updating the Software

- **Docker (Raspberry Pi OS):** Open a terminal and run `update-pib` to update the backend and frontend containers.
- **Native (Ubuntu 24.04):** Update the repositories (e.g. `git pull` in `~/app/pib-backend` and `~/app/cerebra`) and restart the relevant systemd services as needed.

## Webots

Starting the webots simulation:

1. Complete all steps of the "Installing pibs software"-section of this readme document
2. If you are running simulation in a virtual machine ubuntu type the following command
   `xhost +local:root`
4. Navigate to app/pib-backend
   `cd app/pib-backend`
5. Enter the following command into a terminal:  
   `sudo docker compose --profile pibsim_webots up`  
   (The first time this command is entered, webots will be installed. Webots should open automatically afterwards, to close it you should stop the container by closing the terminal window which is open or by pressing ctrl + c. To run it again just restart the container and if you turned off the virtual machine repeat step 2)

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

### Contributing to pib

For the development process, external developers are requested to refer to the following explanation: https://pib-rocks.atlassian.net/wiki/spaces/kb/pages/435486721/Contributing+to+pib
