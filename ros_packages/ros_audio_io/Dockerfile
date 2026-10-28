FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# pip runs as root in an image build; silence the warning it prints for every install.
ENV PIP_ROOT_USER_ACTION=ignore \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# Package postinsts call systemd-tmpfiles, which cannot resolve the systemd users inside an
# image build ("Failed to resolve user 'systemd-network'", dozens of times). Nothing in this
# container is started by systemd, so the step is a no-op for the whole build.
RUN dpkg-divert --local --rename --add /usr/bin/systemd-tmpfiles \
    && ln -s /bin/true /usr/bin/systemd-tmpfiles

RUN apt-get update && apt-get install --no-install-recommends -y curl

# Install necessary dependencies
RUN apt-get update --fix-missing && \
    apt-get install --no-install-recommends -y \
        python3-pip \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-std-msgs \
        python3-usb \
        portaudio19-dev \
        build-essential \
        python3-colcon-mixin \
        pulseaudio \
        libpulse-dev \
        libasound2-dev \
        cmake \
        git \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# PR-2085: Ubuntu's libportaudio has no Pulse host API; PyAudio links it dynamically.
RUN git clone --depth 1 https://github.com/PortAudio/portaudio.git /tmp/portaudio \
    && python3 -c "from pathlib import Path; p=Path('/tmp/portaudio/src/common/pa_front.c'); t=p.read_text(); o='        result = paHostApiInitializers[i]( &hostApis_[hostApisCount_], hostApisCount_ );\\n        if( result != paNoError )\\n            goto error;'; n='        result = paHostApiInitializers[i]( &hostApis_[hostApisCount_], hostApisCount_ );\\n        if( result != paNoError )\\n        {\\n            hostApis_[hostApisCount_] = NULL;\\n            result = paNoError;\\n            continue;\\n        }'; assert o in t, 'PortAudio pa_front.c changed upstream'; p.write_text(t.replace(o,n,1))" \
    && cmake -S /tmp/portaudio -B /tmp/portaudio/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DPA_USE_PULSEAUDIO=ON \
        -DPA_USE_JACK=OFF \
    && cmake --build /tmp/portaudio/build -j"$(nproc)" \
    && cmake --install /tmp/portaudio/build \
    && ldconfig \
    && rm -rf /tmp/portaudio

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# --ignore-installed: avoid conflict with system numpy (installed by ros:jazzy-ros-core)
# Pinned, one resolve for all third-party packages (see voice_assistant/Dockerfile).
RUN pip install --break-system-packages --ignore-installed \
        pyaudio==0.2.14 \
        numpy==2.5.3 \
        scipy==1.18.1 \
        taskgroup==0.2.2

# PR-2085: fail the build when PortAudio inside PyAudio has no Pulse host API.
RUN pulseaudio --daemonize=true --exit-idle-time=-1 --system=false --disallow-exit \
    && python3 -c "import pyaudio; p=pyaudio.PyAudio(); apis=[p.get_host_api_info_by_index(i)['name'] for i in range(p.get_host_api_count())]; assert any('pulse' in n.lower() for n in apis), apis" \
    && pulseaudio -k

COPY ./pib_api/client ./client
COPY ./pib_hermes_config /tmp/pib_hermes_config
# In-repo packages without dependencies; --no-deps keeps pip from re-resolving the environment.
RUN pip install --break-system-packages --no-deps ./client /tmp/pib_hermes_config

COPY ./ros_packages/datatypes ros2_ws/datatypes
COPY ./ros_packages/ros_audio_io ros2_ws/ros_audio_io

RUN cd ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build

COPY ./ros_packages/ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
