from pib_api.flask.app.app import db


class AssistantModel(db.Model):

    __tablename__ = "assistant_model"

    id = db.Column(db.Integer, primary_key=True)
    api_name = db.Column(db.String(255), nullable=False, unique=False)
    visual_name = db.Column(db.String(255), nullable=False, unique=True)
    has_image_support = db.Column(db.Boolean, nullable=False, default=False)
