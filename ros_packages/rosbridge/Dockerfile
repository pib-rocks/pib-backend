FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

# apt-get, not apt: apt warns "does not have a stable CLI interface" on every call in a build.
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    ros-$ROS_DISTRO-rosbridge-server && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# rosapi 2.7.1 is the Jazzy release (rosdistro 2.7.1-1; the apt pool has no
# newer package). Its parameter-client cache starts last_used_time at Time()
# (SYSTEM_TIME) while cleanup subtracts the node clock (ROS_TIME), which kills
# rosapi_node. Upstream jazzy commit 4da27bb4266c7feb2dc09e3778bec94827e07814
# measures that lifetime with time.monotonic() but is not bloom-released.
# Rewrite only the known constructor onto the node clock, and fail the build
# on any other source shape.
COPY ./rosbridge/patch_rosapi_param_clock.py /opt/pib/patch_rosapi_param_clock.py
RUN chmod +x /opt/pib/patch_rosapi_param_clock.py && \
    /opt/pib/patch_rosapi_param_clock.py --ros-root "/opt/ros/${ROS_DISTRO}"

WORKDIR /app

# rosbridge builds the request for a `call_service` from the service's .srv type, so every
# interface package the MCP tools call over the websocket has to be built in this image.
# Without button_service, "/tf_button/set_color" failed before it reached the node with
# "Unable to import button_service.srv from package button_service" - while the same call
# answered success=True inside the container that runs the node.
COPY ./datatypes ros2_ws/datatypes
COPY ./button_service ros2_ws/button_service

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
