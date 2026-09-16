FROM python:3.10-slim

WORKDIR /app

ARG APP_VERSION
# Compose/build-arg supplies the release tag; empty ARG (plain docker build) falls back to dev.
RUN printf '%s\n' "${APP_VERSION:-dev}" > /etc/pib_version

COPY ./pib_api/flask/requirements.txt /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install /tmp/pib_blockly_client

# Shared with the voice assistant so both resolve the same Hermes profiles dir.
COPY ./pib_hermes_config /tmp/pib_hermes_config
RUN pip install /tmp/pib_hermes_config

# The personality service asks the Hermes daemon to provision a complete profile
# when a personality is created, so the client package must be importable here.
# Without it the Flask app (and its build-time db upgrade) cannot start at all.
COPY ./public_api_client /tmp/public_api_client
RUN pip install /tmp/public_api_client

RUN pip install -r requirements.txt

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db

CMD ["flask", "--app", "run", "run"]
