FROM python:3.10-slim

WORKDIR /app

ARG APP_VERSION

COPY ./pib_api/flask/requirements.txt /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install /tmp/pib_blockly_client

# Shared with the voice assistant so both resolve the same Hermes profiles dir.
COPY ./pib_hermes_config /tmp/pib_hermes_config
RUN pip install /tmp/pib_hermes_config

RUN pip install -r requirements.txt

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db

RUN printf 'APP_VERSION="%s"\n' "$APP_VERSION" > /app/version.py

CMD ["flask", "--app", "run", "run"]
