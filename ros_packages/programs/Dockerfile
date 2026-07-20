FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
COPY ./pib_api/client /tmp/pib_api_client
COPY ./ros_packages/motors/pib_motors /tmp/pib_motors
COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/programs ros2_ws/programs
COPY ./ros_packages/button_service ros2_ws/button_service

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# --ignore-installed: avoid conflict with system numpy (installed by ros:jazzy-ros-core)
RUN pip install --break-system-packages --no-cache-dir --ignore-installed \
        /tmp/pib_blockly_client \
        /tmp/pib_api_client \
        /tmp/pib_motors \
        requests==2.32.3 \
        paramiko==3.5.1 \
        tinkerforge \
        opencv-python-headless \
        "depthai<3" \
        blobconverter

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
