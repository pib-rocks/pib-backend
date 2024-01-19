from model.bricklet_model import Bricklet

def get_all_bricklets():
    return Bricklet.query.all()