FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    ros-$ROS_DISTRO-rosbridge-server&& \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./datatypes ros2_ws/datatypes

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
