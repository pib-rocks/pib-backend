# 3.11 matches the unit-test job. hermes-agent (below) refuses Python 3.10.
FROM python:3.11-slim

WORKDIR /app

# pip runs as root in an image build; silence the warning it prints for every install.
ENV PIP_ROOT_USER_ACTION=ignore \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

ARG APP_VERSION
# Compose/build-arg supplies the release tag; empty ARG (plain docker build) falls back to dev.
RUN printf '%s\n' "${APP_VERSION:-dev}" > /etc/pib_version

# The pinned third-party set first, in one resolve.
COPY ./pib_api/flask/requirements.txt /app
RUN pip install -r requirements.txt

# hermes_constants and hermes_cli are import-safe (stdlib only). --no-deps
# keeps this from re-resolving Flask's pinned set; hermes-agent pins a newer
# requests than this image. The voice image installs the same version with
# its own dependency set.
RUN pip install --no-deps hermes-agent==0.19.0

# In-repo packages without dependencies; --no-deps keeps pip from re-resolving the environment.
COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
# Shared with the voice assistant so both resolve the same Hermes profiles dir.
COPY ./pib_hermes_config /tmp/pib_hermes_config
RUN pip install --no-deps /tmp/pib_blockly_client /tmp/pib_hermes_config

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db

CMD ["flask", "--app", "run", "run"]
