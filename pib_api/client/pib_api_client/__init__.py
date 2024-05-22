import os
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import json
import logging

logging.basicConfig(level=logging.INFO,
                    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s")

URL_PREFIX = os.getenv("FLASK_API_BASE_URL", "http://localhost:5000")


def send_request(request: Request) -> (bool, dict[str, Any]):
    try:
        with urlopen(request) as response:
            response_data = response.read().decode('utf-8')
            response_json = json.loads(response_data)
        return True, response_json

    except HTTPError as error:

        logging.error(
            f"Error sending HTTP-Request, received following response from server: \n" +
            f"\tstatus: {error.code},\n" +
            f"\treason: {error.reason},\n" +
            f"\tresponse-body: {error.fp.read().decode()}" +
            f"Request that caused the error:\n" +
            f"\turl: {request.full_url},\n" +
            f"\tmethod: {request.method},\n" +
            f"\tbody: {request.data},\n" +
            f"\theaders: {request.header_items()}"
        )

    except Exception as error:
        logging.error(f"Error communicating with pib API: {error}")

    return False, None
