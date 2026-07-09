#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export TF_HOST=host.docker.internal
export TF_PORT=4223
export TF_BUTTON_UIDS=2df5,2dvS,2dgb

ros2 run button_service button_service_node
