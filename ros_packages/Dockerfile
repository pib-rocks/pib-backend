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
        wget \
        python3-rosdep

# OAK-D-LITE CAMERA SETUP (TAKEN FROM: https://github.com/luxonis/depthai-ros/blob/humble/Dockerfile)
# Clone depthai-python repository
WORKDIR /
RUN git clone --recurse-submodules https://github.com/luxonis/depthai-python.git
WORKDIR /depthai-python/examples

# Install depthai-python requirements
RUN python3 /depthai-python/examples/install_requirements.py

# Clone and install depthai_hand_tracker
WORKDIR /
RUN git clone https://github.com/geaxgx/depthai_hand_tracker.git
WORKDIR /depthai_hand_tracker
RUN pip install -r requirements.txt

WORKDIR /ros_ws

# Python Dependencies for all ros_packages
COPY ./ros_packages/requirements.txt .
RUN pip install -r requirements.txt
RUN pip install --upgrade google-cloud-speech

RUN pip install tinkerforge
COPY ./pib_api/client ./client
RUN pip install ./client/

COPY ./ros_packages/motors/pib_motors ./motors/pib_motors
RUN pip install ./motors/pib_motors

COPY ./ros_packages/voice_assistant/pib_voice ./voice_assistant/pib_voice
RUN pip install ./voice_assistant/pib_voice

# Copy ros_packages
COPY ./ros_packages/datatypes ./datatypes
COPY ./ros_packages/voice_assistant ./voice_assistant
COPY ./ros_packages/programs ./programs
COPY ./ros_packages/camera ./camera
COPY ./ros_packages/motors ./motors

COPY ./ros_packages/pib_launch.yaml .
COPY ./ros_packages/docker_run_all_nodes.sh .
CMD /bin/bash -c 'source docker_run_all_nodes.sh'