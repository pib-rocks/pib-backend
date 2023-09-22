# motors
This repository is for ros package "motors" including one node "motor_control". This node subscribes to "motor_settings" topic and recieves messages from cerebra web application on local host. Followingly the recieved message is split into motor name and value storing them in variables. Motor pin and bricklet retrieved from 2 dictionaries, one maping motors the connected servo bricklet (3 bricklets used) and one maps motor pin connected on servo bricklet. After each message is retrieved an if conditions decides which servo bricklet is the motor connected to according to the number its maped to in the first dictionary. Then using Tinkerforge APIs, motors are moved with the value in message applied on pin that is mapped to motor name.

**Please access node directly to edit, navigate to motors/motor_control.py. Change UID, UID1, UID2 and UID3 of hat brick and bricklets retrieved from brickviewer (lines 3,4,5,6 in code)**

```
HOST = "localhost"
PORT = 4223
UID1 = "XYZ" # Replace with the UID of first Servo Bricklet
UID2 = "XYZ" # Replace with the UID of second Servo Bricklet
UID3 = "XYZ" # Replace with the UID of third Servo Bricklet
```

To run the node:
```
#Navigate to your ros workspace "cd your_ROS_ws" or create one if you don't have it yet "mkdir ros2_ws"
git clone https://github.com/pib-rocks/motors.git
colcon build --packages-select motors
cd install
source setup.bash
cd ..
ros2 run motors motor_control
```
Then navigate to local host from a web browser, moving the slider should print on terminal the shown message and if connected to hardware should move motors directly. (Error device has been replaced is given when you are not connecting to motors and only printing to terminal)

