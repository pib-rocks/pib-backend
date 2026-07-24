FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install --no-install-recommends -y curl

RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        build-essential \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        pulseaudio \
        portaudio19-dev \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_packages/requirements.txt .

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
RUN pip install --break-system-packages --no-cache-dir --ignore-installed requests pyaudio google-genai supertonic onnxruntime scipy soundfile

ENV SUPERTONE_MODEL_PATH=/data/voice/models/supertone/

COPY ./pib_api/client ./client
RUN pip install --break-system-packages --no-cache-dir ./client

COPY ./public_api_client ./public_api_client
RUN pip install --break-system-packages --no-cache-dir ./public_api_client/

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/voice_assistant ros2_ws/voice_assistant

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

RUN export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
