# Hermes turn daemon: runs the agent in-process on the Python version it requires.
#
# Why this image exists: the agent's dependencies are gated to 3.14 and its lock
# covers 3.14 only, while the ROS image is 3.12 - so the daemon cannot run inside
# the voice container. As a dedicated compose service it gets service-name DNS,
# the existing egress policy and a reproducible 3.14 environment.
#
# The agent *sources* are not baked in: they come from the filtered Hermes view
# that the voice container already mounts (prepare-view publishes profiles and
# hermes-agent, and covers the .env files). Only the dependency set is installed
# here, from the agent's own pyproject.toml/uv.lock via the "hermes" build context.
FROM python:3.14-slim

ENV PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_ROOT_USER_ACTION=ignore \
    PYTHONUNBUFFERED=1 \
    HERMES_DISABLE_LAZY_INSTALLS=1

RUN apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends curl build-essential >/dev/null \
    && rm -rf /var/lib/apt/lists/*

# The agent's own dependency set, resolved by its lock (covers 3.14 only).
COPY --from=hermes pyproject.toml uv.lock /src/
RUN pip install uv >/dev/null 2>&1 \
    && cd /src \
    && uv sync --frozen --no-install-project \
    && rm -rf /root/.cache

WORKDIR /app

# In-repo packages the daemon imports: the turn itself, the profile helpers, the
# key-store client, the MCP server and the direct tool loop.
COPY public_api_client/ /app/public_api_client/
COPY pib_hermes_config/ /app/pib_hermes_config/
COPY pib_api/client/ /app/pib_api_client/
COPY pib_mcp_server/ /app/pib_mcp_server/
COPY ros_packages/voice_assistant/voice_assistant /app/voice_assistant/

# pyyaml/requests are read by public_api_client; mcp and websocket-client are the
# declared dependencies of pib_mcp_server (the agent's default group excludes them).
#
# A personality's mcp_servers.pib entry runs "/usr/bin/python3 -m pib_mcp_server",
# and an MCP child receives only the env that entry names - so neither PATH nor
# PYTHONPATH reaches it. Make /usr/bin/python3 a wrapper into /src/.venv and put
# the repo packages into its site-packages via a .pth file so profiles work unchanged.
RUN VIRTUAL_ENV=/src/.venv uv pip install --no-cache \
        pyyaml requests "mcp==2.3.0" "websocket-client==1.9.2" \
    && rm -f /usr/bin/python3 /usr/bin/python3.14 \
    && printf '#!/bin/sh\nexec /src/.venv/bin/python3 "$@"\n' > /usr/bin/python3 \
    && chmod 0755 /usr/bin/python3 \
    && printf '%s\n' /app /app/public_api_client /app/pib_hermes_config /app/pib_api_client /app/pib_mcp_server \
       > /src/.venv/lib/python3.14/site-packages/pib.pth \
    && TRYB_URL_PREFIX=https://platform.tryb.ai /usr/bin/python3 -c "import pib_mcp_server, mcp, yaml, public_api_client, pib_hermes_config, pib_api_client; print('all imports ok')"

ENV PYTHONPATH=/app:/app/public_api_client:/app/pib_hermes_config:/app/pib_api_client:/app/pib_mcp_server:/home/pib/.hermes/hermes-agent:/src/.venv/lib/python3.14/site-packages

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
    CMD curl -fsS http://127.0.0.1:8088/health || exit 1

CMD ["/src/.venv/bin/python3", "-m", "public_api_client.hermes_daemon"]
