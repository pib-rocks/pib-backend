FROM python:3.10-slim

WORKDIR /app

COPY ./pib_api/flask/requirements.txt /app

COPY ./pib_blockly/pib_blockly_client /tmp/pib_blockly_client
RUN pip install --break-system-packages /tmp/pib_blockly_client

RUN pip install --break-system-packages -r requirements.txt

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db


CMD ["flask", "--app", "run", "run"]
