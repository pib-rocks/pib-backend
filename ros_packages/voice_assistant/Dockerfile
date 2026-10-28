FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# pip runs as root in an image build; silence the warning it prints for every install.
ENV PIP_ROOT_USER_ACTION=ignore \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# Force IPv4 preference in glibc getaddrinfo to prevent 10s IPv6 route timeout on Pi
RUN echo "precedence ::ffff:0:0/96 100" >> /etc/gai.conf

# Package postinsts call systemd-tmpfiles, which cannot resolve the systemd users inside an
# image build ("Failed to resolve user 'systemd-network'", dozens of times). Nothing in this
# container is started by systemd, so the step is a no-op for the whole build.
RUN dpkg-divert --local --rename --add /usr/bin/systemd-tmpfiles \
    && ln -s /bin/true /usr/bin/systemd-tmpfiles

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

COPY ./ros_packages/voice_assistant/requirements.txt .

# PEP 668: Jazzy (Ubuntu 24.04, Python 3.12) requires --break-system-packages
# One resolve for every third-party package, pinned: a second `pip install` on top of the
# first is what makes pip report "dependency resolver does not currently take into account
# all the packages that are installed". hermes-agent pins requests and (in its mcp extra)
# mcp, so both are pinned to the same versions here; mcp and websocket-client are the
# runtime dependencies of pib_mcp_server, installed below with --no-deps.
# --ignore-installed stays because the apt-managed ROS Python packages (egg-info, no RECORD)
# cannot be uninstalled by pip and must be shadowed instead.
#
# av is pinned because faster-whisper 1.2.1 decodes through
# ``av.open(..., metadata_errors="ignore")``, and PyAV 19.0.0 REMOVED that keyword
# (changelog 19.0.0: "Remove the metadata_encoding and metadata_errors arguments to
# av.open()"). An unpinned av resolved to 19.0.1, so every local transcription raised
# ``open() got an unexpected keyword argument 'metadata_errors'`` and produced an empty
# text - the defect PC-35 fixes. 18.1.0 is the newest PyAV that still accepts the
# keyword; it also satisfies faster-whisper's own ``av>=11`` requirement. Raise this
# pin only together with a faster-whisper version that dropped the keyword.
RUN pip install --break-system-packages --ignore-installed \
        requests==2.33.0 \
        httpx==0.28.1 \
        pyaudio==0.2.14 \
        google-genai==2.28.0 \
        supertonic==1.3.1 \
        faster-whisper==1.2.1 \
        av==18.1.0 \
        onnxruntime==1.30.0 \
        scipy==1.18.1 \
        soundfile==0.14.0 \
        hermes-agent==0.19.0 \
        mcp==1.26.0 \
        websocket-client==1.9.2

ENV SUPERTONE_MODEL_PATH=/data/voice/models/supertone/
# Bundled STT decode self-check: on in CI (PIB_STT_SELF_CHECK=1), off at robot start.
ENV PIB_STT_SELF_CHECK=0
ENV WHISPER_MODEL_SIZE=small
ENV WHISPER_MODEL_PATH=/data/voice/models/whisper/
# Language the local engine transcribes in. Pinned, because the engine's own
# detection runs a pass over 99 languages on every turn and cost about half the
# turn on the test robot (PC-39, 8 s recording: 37.0 s -> 19.2 s). Set
# STT_LANGUAGE=auto to hand that decision back to the engine.
ENV STT_LANGUAGE=de

COPY ./pib_api/client ./client
# Shared with the flask API so both resolve the same Hermes profiles dir.
COPY ./pib_hermes_config ./pib_hermes_config
COPY ./public_api_client ./public_api_client

# Chat resolves the cloud model from the catalogue. This image does not
# install Flask; the module itself imports nothing else.
COPY ./pib_api/flask/provider_registry.py /tmp/provider_registry.py
RUN cp /tmp/provider_registry.py \
    "$(python3 -c 'import sysconfig; print(sysconfig.get_path("purelib"))')/provider_registry.py"

COPY ./pib_mcp_server ./pib_mcp_server
# The in-repo packages declare no dependencies beyond what is installed above (pib_mcp_server:
# mcp, websocket-client). --no-deps keeps this layer from re-resolving the environment and
# reporting conflicts in it.
RUN pip install --break-system-packages --no-deps \
        ./client \
        ./pib_hermes_config/ \
        ./public_api_client/ \
        ./pib_mcp_server/

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
