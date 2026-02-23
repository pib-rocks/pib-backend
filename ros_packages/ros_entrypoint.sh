#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/ros2_ws/install/setup.bash

exec "$@"