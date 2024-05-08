FROM ros:humble-ros-core

WORKDIR /ros_ws

RUN apt-get update && apt-get install -y curl

# Get system packages + add Tinkerforge Repository
RUN curl -sSL https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg | apt-key add - && \
    echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" > /etc/apt/sources.list.d/tinkerforge.list && \
    apt-get update --fix-missing && \
    apt-get install -y \
        python3-tinkerforge \
        python3-colcon-common-extensions \
        python3-pip \
        portaudio19-dev \
        ros-humble-ament-cmake \
        ros-humble-rosbridge-server \
        ros-humble-tf2-msgs \
        libusb-1.0-0-dev \
        flac \
        libgl1 \
        zsh \
        git \
        software-properties-common \
        wget

# OAK-D-LITE CAMERA SETUP (TAKEN FROM: https://github.com/luxonis/depthai-ros/blob/humble/Dockerfile)
# RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
# ENV WS=/ws
# RUN mkdir -p $WS/src
# COPY ./ .$WS/src/depthai-ros
# RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y
# RUN cd .$WS/ && . /opt/ros/humble/setup.sh && ./src/depthai-ros/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1 
# RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-humble-rviz2 ros-humble-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
# RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
# RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
# RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
# RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc

# Python Dependencies for all ros_packages
RUN pip install tinkerforge
COPY ./pib_api/client ./client
RUN pip install ./client/

COPY ./ros_packages/motors/pib_motors ./motors/pib_motors
RUN pip install ./motors/pib_motors

COPY ./public_api_client /tmp/public_api_client
RUN pip install /tmp/public_api_client

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install /tmp/pib_blockly_client

COPY ./ros_packages/requirements.txt .
RUN pip install -r requirements.txt
RUN pip install --upgrade google-cloud-speech


# Copy ros_packages
COPY ./ros_packages/datatypes ./datatypes
COPY ./ros_packages/voice_assistant ./voice_assistant
COPY ./ros_packages/programs ./programs
COPY ./ros_packages/camera ./camera
COPY ./ros_packages/motors ./motors
COPY ./ros_packages/pib_launch.yaml .


COPY ./ros_packages/docker_run_all_nodes.sh .
CMD /bin/bash -c 'source docker_run_all_nodes.sh'