#!/bin/bash
set -e

/app/ros2_ws/install/voice_assistant/share/voice_assistant/langchain/langchain_proxy.py &

source /opt/ros/humble/setup.bash
source /app/ros2_ws/install/setup.bash

exec "$@"