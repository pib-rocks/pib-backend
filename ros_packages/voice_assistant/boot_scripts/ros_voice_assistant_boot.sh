#!/bin/bash
source /home/pib/ros_working_dir/ros_config.sh
source /opt/ros/jazzy/setup.bash
source /home/pib/ros_working_dir/install/setup.bash

# Chat resolves the cloud model from the catalogue module. The voice image
# installs that file; a checkout keeps it beside the API.
catalogue=""
for candidate in \
    /home/pib/app/pib-backend/pib_api/flask \
    "${HOME}/flask" \
    /home/pib/flask
do
    if [[ -f "${candidate}/provider_registry.py" ]]; then
        catalogue="${candidate}"
        break
    fi
done
if [[ -n "${catalogue}" ]]; then
    export PYTHONPATH="${catalogue}${PYTHONPATH:+:${PYTHONPATH}}"
fi

ros2 launch voice_assistant launch.py 2>&1 | tee -a ~/ros_working_dir/src/voice_assistant/assistant.log