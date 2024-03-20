# Docker setup

This script assumes: 
- that you are working on a linux distribution with docker support
- prerequisted packages:
    - docker.io
    - docker-compose
    - nvidia-docker2 (if you have a nvidia graphics card)

- download the docker-compose.yml file
- Go to the directory of the docker-compose.yml file
- start the following command:

	docker-compose up 

- if everything succeeds a window should open with the simulation of our
  so called digital twin of pib.


# Containers

1.) container pib-digital-twin
  - is a Ubuntu 22.04 with a full installed ros2 humble desktop and webots robot simulator
  - a control node is started for pib and pib's digital twin is loaded into the webots simulator

2.) container pib-backend-bridge
  - is a Ubuntu 22.04 with a full installed ros2 humble desktop and ros-humble-rosbridge-server
  - the rosbridge server is started

3.) container cerebra
  - is a debian based image with an installed nginx webserver
  - The webserver is started with a controlling website for pib
  - the portnumber to access it is 8000, so if you brwose to http://localhost:8000 on your host
    you should see the controlling page for pib

The whole system is still under development. The cerebra controlling software is not yet connected to
the simulator.

To test the simulator you can do the following steps:

1.) Open a new terminal while the docker-compose command is still running.
2.) Run the following command:

    docker exec -it pib-digitaltwin-webots /bin/bash 

3.) Now you are inside the container. Here you run

    . /opt/ros/humble/setup.bash
    . /home/ros_ws/install/setup.bash
    rqt

4.) In the new window click on "Plugins"->"Topics"->"Message Publisher"

5.) Set Topic to "/pib/cmd_vel" and click on "+" sign

6.) Expand the "pib/cmd_val" on teh little triangle and again the "angular" line

7.) Set the z value from "0.0" to e.g. "1.0" and set the check mark on the "/pib/cmd_vel" line.

8.) Now you should see that the head of pib should turn inside the simulation ;-)