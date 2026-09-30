from sqlalchemy import Index, text

from app.app import db


class Provider(db.Model):
    """Registry row that extends one assistant_model, keeping its id."""

    __tablename__ = "provider"
    __table_args__ = (
        Index(
            "uq_provider_single_default",
            "is_default",
            unique=True,
            sqlite_where=text("is_default = 1"),
        ),
    )

    id = db.Column(db.Integer, primary_key=True)
    api_name = db.Column(db.String(255), nullable=False)
    visual_name = db.Column(db.String(255), nullable=False, unique=True)
    has_image_support = db.Column(db.Boolean, nullable=False, default=False)
    endpoint_base = db.Column(db.String(1024), nullable=True)
    capabilities = db.Column(db.JSON, nullable=False)
    credential_ref = db.Column(db.String(255), nullable=True)
    is_default = db.Column(db.Boolean, nullable=False, default=False)
    # Pinned against the account model list. Null until that list has been read
    # and the candidate was present. The live capability flag gates its use.
    live_model = db.Column(db.String(255), nullable=True)
    live_model_checked_on = db.Column(db.Date(), nullable=True)
