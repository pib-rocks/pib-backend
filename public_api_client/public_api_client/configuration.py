import json
import os

try:
    with open("/home/pib/public_api/config.json") as f: _data = f.read()
    _config = json.loads(_data)
    tryb_url_prefix = _config["trybUrlPrefix"]
    public_api_token = _config["publicApiToken"]
    
except FileNotFoundError:
    tryb_url_prefix = os.getenv("TRYB_URL_PREFIX")
    public_api_token = os.getenv("PUBLIC_API_TOKEN")

