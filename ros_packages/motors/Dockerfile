FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install --no-install-recommends -y curl

# Tinkerforge apt repository (GPG key via signed-by, not deprecated apt-key)
RUN curl -sSL https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg -o /usr/share/keyrings/tinkerforge.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/tinkerforge.gpg] https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" > /etc/apt/sources.list.d/tinkerforge.list && \
    apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        build-essential \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        python3-tinkerforge \
        libusb-1.0-0-dev \
        ros-${ROS_DISTRO}-diagnostic-msgs \
        ros-${ROS_DISTRO}-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
RUN pip install --break-system-packages --no-cache-dir requests

COPY ./pib_api/client ./client
RUN pip install --break-system-packages --no-cache-dir ./client

COPY ./ros_packages/motors/pib_motors ./motors/pib_motors
RUN pip install --break-system-packages --no-cache-dir ./motors/pib_motors

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/button_service ros2_ws/button_service
COPY ./ros_packages/motors ros2_ws/motors

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
