FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install /tmp/pib_blockly_client

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/programs ros2_ws/programs

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]