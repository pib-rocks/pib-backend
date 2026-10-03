from sqlalchemy import Index, text

from app.app import db


class Provider(db.Model):
    """One account. The credential, the endpoint, and the flags every model shares."""

    __tablename__ = "provider"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False, unique=True)
    endpoint_base = db.Column(db.String(1024), nullable=True)
    capabilities = db.Column(db.JSON, nullable=False)
    credential_ref = db.Column(db.String(255), nullable=True)
    models = db.relationship(
        "RegistryModel",
        back_populates="provider",
        lazy=True,
    )


class RegistryModel(db.Model):
    """One model of a provider: the model id, the display name, and its own flags.

    A personality stores this row's id. The provider follows from provider_id.
    """

    __tablename__ = "registry_model"
    __table_args__ = (
        Index(
            "uq_registry_model_single_default",
            "is_default",
            unique=True,
            sqlite_where=text("is_default = 1"),
        ),
    )

    id = db.Column(db.Integer, primary_key=True)
    provider_id = db.Column(db.Integer, db.ForeignKey("provider.id"), nullable=False)
    api_name = db.Column(db.String(255), nullable=False)
    visual_name = db.Column(db.String(255), nullable=False, unique=True)
    has_image_support = db.Column(db.Boolean, nullable=False, default=False)
    capabilities = db.Column(db.JSON, nullable=False)
    is_default = db.Column(db.Boolean, nullable=False, default=False)
    # Pinned against the account model list. Null until that list has been read
    # and the candidate was present. The live capability flag gates its use.
    live_model = db.Column(db.String(255), nullable=True)
    live_model_checked_on = db.Column(db.Date(), nullable=True)
    provider = db.relationship("Provider", back_populates="models")
