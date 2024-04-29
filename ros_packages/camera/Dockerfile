FROM luxonis/depthai-ros:humble-latest
SHELL ["/bin/bash", "-c"]

WORKDIR /app
COPY ./datatypes ros2_ws/datatypes
COPY ./camera ros2_ws/camera

RUN set -xe \
    && apt-get update \
    && apt-get install -y python3-pip

RUN pip install depthai

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]