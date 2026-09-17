from app.app import db


class ButtonProgram(db.Model):
    __tablename__ = "button_program"
    id = db.Column(db.Integer, primary_key=True)
    controller_id = db.Column(
        db.Integer, db.ForeignKey("controller.id"), nullable=False, unique=True
    )
    program_id = db.Column(
        db.Integer, db.ForeignKey("program.id", ondelete="SET NULL"), nullable=True
    )

    controller = db.relationship("Controller", lazy="joined")
    program = db.relationship("Program", lazy="joined")

    @property
    def bricklet(self):
        """Transition alias for clients still using Bricklet terminology."""
        return self.controller

    @property
    def bricklet_id(self):
        return self.controller_id
