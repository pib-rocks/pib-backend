FROM luxonis/depthai-ros:jazzy-latest
SHELL ["/bin/bash", "-c"]

WORKDIR /app
COPY ./datatypes ros2_ws/datatypes
COPY ./camera ros2_ws/camera

RUN set -xe \
    && apt-get update \
    && apt-get install -y python3-pip python3-venv

RUN python3 -m venv /opt/pib-venv

ENV PATH="/opt/pib-venv/bin:$PATH"

RUN pip install --no-cache-dir depthai==2.25.1.0

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]