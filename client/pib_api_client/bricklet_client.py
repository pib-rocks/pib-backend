from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX


GET_ALL_MOTORS_URL = URL_PREFIX + '/motor'
UPDATE_MOTOR_SETTINGS_URL = URL_PREFIX + '/motor/%s/settings'


def get_all_motors() -> (bool, dict[str, Any]):
        request = Request(GET_ALL_MOTORS_URL, method='GET')
        return send_request(request)


def update_motor_settings(motor_name, motor_settings_dto) -> (bool, dict[str, Any]):
        request = Request(
                UPDATE_MOTOR_SETTINGS_URL % motor_name, 
                method='PUT',
                headers = { 'Content-Type': 'application/json' },
                body = json.dumps(motor_settings_dto).encode('UTF-8'))
        return send_request(request)