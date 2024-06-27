"""a client for accessing the pib-blockly-server for compilation of visual-code into python-code"""

from typing import Tuple
import os
import requests
import logging

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s",
)

PIB_BLOCKLY_SERVER_URL = os.getenv("PIB_BLOCKLY_SERVER_URL", "http://localhost:2442")


def code_visual_to_python(code_visual: str) -> Tuple[bool, str | None]:
    """
    compile the provided visual-code into python-code via the pib-blockly-server.

    Returns a bool indicating if compilation was succesful and, in case of a success,
    the rsulting python-code.
    """

    try:
        response = requests.request(
            method="POST",
            url=PIB_BLOCKLY_SERVER_URL,
            headers={"Content-Type": "text/plain"},
            data=code_visual.encode(),
        )
        response.raise_for_status()
        code_python = response.text
        return True, code_python

    except requests.HTTPError as error:
        logging.error(
            f"pib-blockly-server responded with error '{response.text}' (code: {response.status_code})"
        )

    except Exception as error:
        logging.error(f"unexpected error occured: {error}.")

    return False, None
