FROM ros:jazzy-ros-core AS builder

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY ./datatypes ros2_ws/datatypes
COPY ./camera ros2_ws/camera

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

# --- runtime stage (minimal) ---
FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# --ignore-installed: avoid conflict with system numpy (installed by ros:jazzy-ros-core)
# opencv-python-headless: stereo.py imports cv2 directly (was missing from the
# runtime stage — only depthai was installed, causing "ModuleNotFoundError: cv2"
# and a container restart loop).
RUN pip install --break-system-packages --no-cache-dir --ignore-installed \
        depthai==2.25.1.0 \
        opencv-python-headless

WORKDIR /app

COPY --from=builder /app/ros2_ws/install /app/ros2_ws/install

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
