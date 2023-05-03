# General Info

ROS2 Python node for [OAK-D-Lite](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095.html) stereo camera

# Installation

1. Install [ros2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)

2. Install depthai python library:

                pip install depthai

3. Clone this repository into ros2:

                git clone https://github.com/pib-rocks/ros2_oak_d_lite.git

4. Build the with:


                cd ros2_oak_d_lite
                colcon build --packages-select ros2_oak_d_lite


5. source the overlay with:

                source install/local_setup.bash

# Run:

                ros2 run ros2 run oak_d_lite stereo
