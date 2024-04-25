import json
import os

config_file_path = os.getenv("PUBLIC_API_CONFIG_FILE_PATH", "/home/pib/public_api/config.json")
with open(config_file_path) as f: _data = f.read()
_config = json.loads(_data)

tryb_url_prefix = _config["trybUrlPrefix"]
public_api_token = _config["publicApiToken"]