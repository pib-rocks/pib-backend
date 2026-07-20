FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-gi \
    libgirepository-1.0-1 \
    gir1.2-gtk-4.0 \
    libgtk-4-1 \
    libwayland-client0 \
    libwayland-egl1 \
    libwayland-cursor0 \
    libdecor-0-0 \
    libxkbcommon0 \
    libegl1 \
    libgles2 \
    libdrm2 \
    libdbus-1-3 \
    libglib2.0-0 \
    fonts-dejavu \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /app

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
RUN pip install --break-system-packages --no-cache-dir requests==2.32.3 pillow

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/display ros2_ws/display

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
