from model.bricklet_model import Bricklet
from app.app import db


def get_all_bricklets() -> list[Bricklet]:
    return Bricklet.query.all()


def get_bricklet(bricklet_number: int) -> Bricklet:
    return Bricklet.query.filter(Bricklet.brickletNumber == bricklet_number).one()


def set_bricklet_uid(bricklet_number: int, uid: str) -> Bricklet:
    bricklet = get_bricklet(bricklet_number)
    bricklet.uid = uid
    db.session.flush()
    return bricklet
