FROM python:3.10-slim

ENV VIRTUAL_ENV=/opt/pib-venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

WORKDIR /app

COPY ./pib_api/flask/requirements.txt /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install --no-cache-dir /tmp/pib_blockly_client

RUN pip install --no-cache-dir -r requirements.txt

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db


CMD ["flask", "--app", "run", "run"]
