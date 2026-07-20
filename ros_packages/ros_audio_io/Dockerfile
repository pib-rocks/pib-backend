FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install --no-install-recommends -y curl

# Install necessary dependencies
RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        python3-pip \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-std-msgs \
        python3-usb \
        portaudio19-dev \
        build-essential \
        python3-colcon-mixin \
        pulseaudio \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# --ignore-installed: avoid conflict with system numpy (installed by ros:jazzy-ros-core)
RUN pip install --break-system-packages --no-cache-dir --ignore-installed pyaudio numpy scipy taskgroup

COPY ./pib_api/client ./client
RUN pip install --break-system-packages --no-cache-dir ./client

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/ros_audio_io ros2_ws/ros_audio_io

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
