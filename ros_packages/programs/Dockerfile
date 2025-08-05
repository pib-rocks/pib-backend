FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-colcon-common-extensions \
    python3-pip \
    python3-dev \
    libatlas-base-dev \
    liblapack-dev \
    gfortran \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
COPY ./pib_api/client /tmp/pib_api_client
RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y python3-pip && \
    pip install /tmp/pib_blockly_client && \
    pip install /tmp/pib_api_client && \
    pip install requests==2.32.3

RUN pip install --upgrade pip setuptools wheel
RUN pip install numpy scipy matplotlib spatialmath-python
RUN pip install --no-binary :all: roboticstoolbox-python==1.1.1
RUN pip install pib-sdk

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/programs ros2_ws/programs

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]