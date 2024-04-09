import json

with open("/home/pib/public_api/config.json") as f: _data = f.read()
_config = json.loads(_data)

tryb_url_prefix = _config["trybUrlPrefix"]
public_api_token = _config["publicApiToken"]