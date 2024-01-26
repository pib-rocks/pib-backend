from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX
from  tinkerforge_service import TinkerForge



GET_ALL_BRICKLETS_URL = URL_PREFIX + '/bricklet'
tinkerforge_service = TinkerForge()


def get_all_bricklets() -> (bool, dict[str, Any]):
        request = Request(GET_ALL_BRICKLETS_URL, method='GET')
        return send_request(request)

def update_all_bricklets_on_start() ->():
        tinkerforge_service.get_ids()
        Request(GET_ALL_BRICKLETS_URL+"1", {{"uid": tinkerforge_service.UID0}}, method='PUT')
        Request(GET_ALL_BRICKLETS_URL+"2", {{"uid": tinkerforge_service.UID1}}, method='PUT')
        Request(GET_ALL_BRICKLETS_URL+"3", {{"uid": tinkerforge_service.UID2}}, method='PUT')
        return
    
def detact_changes_between_db_and_tinkerforge() ->():
        usedUIDs = get_all_bricklets()
        print(usedUIDs)
        #tinker.get_ids()
        for i in range(len(usedUIDs)):
                if(usedUIDs[i] != tinkerforge_service.getValue(i)):
                        return True
        return False
