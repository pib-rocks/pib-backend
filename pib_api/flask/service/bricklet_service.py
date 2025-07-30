from typing import List
from model.bricklet_model import Bricklet
from pib_api.flask.app.app import db


def get_all_bricklets() -> List[Bricklet]:
    return Bricklet.query.all()


def get_bricklet(bricklet_number: int) -> Bricklet:
    return Bricklet.query.filter(Bricklet.bricklet_number == bricklet_number).one()


def set_bricklet_uid(bricklet_number: int, uid: str) -> Bricklet:
    bricklet = get_bricklet(bricklet_number)
    bricklet.uid = uid
    db.session.flush()
    return bricklet
