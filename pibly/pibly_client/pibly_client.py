from typing import Tuple
import os
import requests
import logging

logging.basicConfig(level=logging.INFO, 
                    format="[%(levelname)s] [%(asctime)s] [%(process)d] [%(filename)s:%(lineno)s]: %(message)s")

PIBLY_URL = os.getenv("PIBLY_URL", "http://localhost:2442")

def code_visual_to_python(code_visual: str) -> Tuple[bool, str]:
     
	try:
		response = requests.request(
			method='POST', 
			url=PIBLY_URL, 
			headers={'Content-Type': 'text/plain'}, 
			data=code_visual.encode())
		response.raise_for_status()
		code_python = response.text
		return True, code_python
        
	except requests.HTTPError as error:
		logging.error(f"pibly-server responded with error '{response.text}' (code: {response.status_code})")
    
	except Exception as error:
		logging.error(f"unexpected error occured: {error}.")
        
	return False, None