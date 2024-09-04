FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /app

RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y python3-pip && \
    pip install requests==2.32.3 && \
    pip install pillow

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/display ros2_ws/display

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]