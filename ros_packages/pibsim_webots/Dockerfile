FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
	wget gnupg2 lsb-release ca-certificates \
      && rm -rf /var/lib/apt/lists/*

RUN wget -O /tmp/webots.deb \
        https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb     

RUN apt-get update --fix-missing && apt-get install --no-install-recommends -y \ 
        build-essential \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        /tmp/webots.deb \
        ros-humble-webots-ros2 \
    && rm /tmp/webots.deb \
    && rm -rf /var/lib/apt/lists/*

RUN pip install fastmcp 

COPY ./ros_packages/pibsim_webots ros2_ws/pibsim_webots

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
