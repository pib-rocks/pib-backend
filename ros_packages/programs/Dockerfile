FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# pip runs as root in an image build; silence the warning it prints for every install.
ENV PIP_ROOT_USER_ACTION=ignore \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# --ignore-installed: avoid conflict with system numpy (installed by ros:jazzy-ros-core)
# Third-party packages first, in one resolve. Everything is pinned except pib-sdk, which
# takes the newest release (>= keeps it from resolving to one that predates the shared
# rosbridge client of PR-1882). opencv-python-headless stays below 5 for the same reason as
# in camera/Dockerfile (5.x dropped cv2 APIs user programs rely on).
RUN pip install --break-system-packages --ignore-installed \
        pib-sdk>=0.5.5 \
        requests==2.32.3 \
        paramiko==3.5.1 \
        tinkerforge==2.1.32 \
        "opencv-python-headless<5"

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
COPY ./pib_api/client /tmp/pib_api_client
COPY ./ros_packages/motors/pib_motors /tmp/pib_motors
COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/programs ros2_ws/programs
COPY ./ros_packages/button_service ros2_ws/button_service

# In-repo packages without dependencies; --no-deps keeps pip from re-resolving the environment.
RUN pip install --break-system-packages --no-deps \
        /tmp/pib_blockly_client \
        /tmp/pib_api_client \
        /tmp/pib_motors

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
