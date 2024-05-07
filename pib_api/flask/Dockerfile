FROM python:3.10-slim

WORKDIR /app

COPY ./pib_api/flask/requirements.txt /app

COPY ./pibly/pibly_client /tmp/pibly_client
RUN pip install /tmp/pibly_client

RUN pip install -r requirements.txt

COPY ./pib_api/flask/ .

RUN flask --app run db upgrade
RUN flask --app run seed_db


CMD ["flask", "--app", "run", "run"]
