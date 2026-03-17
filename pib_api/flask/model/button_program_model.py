from app.app import db


class ButtonProgram(db.Model):
    __tablename__ = "button_program"
    id = db.Column(db.Integer, primary_key=True)
    bricklet_id = db.Column(
        db.Integer, db.ForeignKey("bricklet.id"), nullable=False, unique=True
    )
    program_id = db.Column(
        db.Integer, db.ForeignKey("program.id", ondelete="SET NULL"), nullable=True
    )
