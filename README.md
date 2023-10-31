# motors
This repository defines the ros package "motors" that includes two nodes: "motor_control" and "motor_current". 
### motor_control
This node subscribes to the "joint_trajectory" topic and offers the "motor_settings" service. 
- **"joint_trajectory" topic :** 
On this topic, the node receives messages from the cerebra web application that describe the next desired position-value of a specified motor. From each message, the name of the motor, as well as the position that the motor is supposed to move to, are extracted. The motor name is then mapped to a motor-object, where the motor's servo-bricklet and its pins on that bricklet are stored. Using the *Tinkerforge API*, the extracted position-value is then applied to the motor's pins.
 - **"motor_settings" service :**
By calling this service, cerebra is able to send motor-settings to a single, specified motor and then receives a message, that informs the application whether the settings have been successfully applied to the specified motor. The message-datatype that the service demands, asks for the name of the motor, that the settings are supposed to be applied to, as well as the various values that may be set, such as minimum/maximum pulse width, minimum/maximum rotation range, period, and so on. The motor-name is then again mapped to a motor-object containing the motor's servo-bricklet and pins. Subsequently, the settings are applied to the found pins. The response datatype features one single boolean-valued field named 'successful', whose value indicates if the node succeeded in applying the desribed settings.
### motor_current
this node publishes to the "motor_current" topic.
- **"motor_current" topic :** here, the "motor_current"-node publishes the electric current values of all the available motors at a particular moment of time. This takes place periodically, in a constant pre-defined time interval. Each message contains a single key-value pair, that maps a motor-name to the electric-current value of the described motor at the message's time of creation.

\
**Please access nodes directly to edit, navigate to motors/motor_control.py. Change UID, UID1, UID2 and UID3 of hat brick and bricklets retrieved from brickviewer (lines 3,4,5,6 in code)**

```
HOST = "localhost"
PORT = 4223
UID1 = "XYZ" # Replace with the UID of first Servo Bricklet
UID2 = "XYZ" # Replace with the UID of second Servo Bricklet
UID3 = "XYZ" # Replace with the UID of third Servo Bricklet
```

To run the nodes:
```
#Navigate to your ros workspace "cd your_ROS_ws" or create one if you don't have it yet "mkdir ros2_ws"
git clone https://github.com/pib-rocks/motors.git
colcon build --packages-select motors
cd install
source setup.bash
cd ..
ros2 run motors motor_control
ros2 run motors motor_current
```
Then navigate to local host from a web browser, moving the slider should print on terminal the shown message and if connected to hardware should move motors directly. (Error device has been replaced is given when you are not connecting to motors and only printing to terminal)

