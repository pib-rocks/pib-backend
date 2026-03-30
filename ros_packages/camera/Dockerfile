FROM ros:jazzy-ros-core
SHELL ["/bin/bash", "-c"]

WORKDIR /app
COPY ./datatypes ros2_ws/datatypes
COPY ./camera ros2_ws/camera

RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        build-essential \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        python3-opencv \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages depthai==2.25.1.0 && \
    pip install --break-system-packages "empy<4.0"

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]