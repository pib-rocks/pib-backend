FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install --no-install-recommends -y curl

RUN curl -sSL https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID)/tinkerforge.gpg | apt-key add - && \
    echo "deb https://download.tinkerforge.com/apt/$(. /etc/os-release; echo $ID $VERSION_CODENAME) main" > /etc/apt/sources.list.d/tinkerforge.list && \
    apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        build-essential \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        python3-tinkerforge \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*


# ToDo - DEV MERGE: COPY ./ros_packages/requirements.txt .
RUN pip install requests

COPY ./pib_api/client ./client
RUN pip install ./client

COPY ./ros_packages/motors/pib_motors ./motors/pib_motors
RUN pip install ./motors/pib_motors

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/motors ros2_ws/motors
RUN pip install serial
RUN pip install pyserial
RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
