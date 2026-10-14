FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# pip runs as root in an image build; silence the warning it prints for every install.
ENV PIP_ROOT_USER_ACTION=ignore \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# Package postinsts call systemd-tmpfiles, which cannot resolve the systemd users inside an
# image build ("Failed to resolve user 'systemd-network'", dozens of times). Nothing in this
# container is started by systemd, so the step is a no-op for the whole build.
RUN dpkg-divert --local --rename --add /usr/bin/systemd-tmpfiles \
    && ln -s /bin/true /usr/bin/systemd-tmpfiles

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
# Pinned, one resolve for all third-party packages (see voice_assistant/Dockerfile).
RUN pip install --break-system-packages --ignore-installed \
        pyaudio==0.2.14 \
        numpy==2.5.3 \
        scipy==1.18.1 \
        taskgroup==0.2.2

COPY ./pib_api/client ./client
COPY ./pib_hermes_config /tmp/pib_hermes_config
# In-repo packages without dependencies; --no-deps keeps pip from re-resolving the environment.
RUN pip install --break-system-packages --no-deps ./client /tmp/pib_hermes_config

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/ros_audio_io ros2_ws/ros_audio_io

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
