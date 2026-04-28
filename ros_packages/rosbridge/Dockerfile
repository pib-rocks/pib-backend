FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    ros-$ROS_DISTRO-rosbridge-server && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages "empy<4.0"

WORKDIR /app

COPY ./datatypes ros2_ws/datatypes

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
