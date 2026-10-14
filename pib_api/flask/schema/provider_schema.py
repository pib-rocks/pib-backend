from marshmallow import fields

from model.provider_model import Provider, RegistryModel
from provider_registry import is_listed_model, model_status
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProviderSchema(SQLAutoWithCamelCaseSchema):
    """A selectable model. providerId is the account the model belongs to.

    The live model is this row when the row is a live model. There is no
    second id pinned beside it.
    """

    class Meta:
        model = RegistryModel
        include_fk = True
        exclude = ("live_model", "live_model_checked_on")

    status = fields.Method("get_status", dump_only=True)
    credential_ref = fields.Method("get_credential_ref", dump_only=True)

    def get_status(self, obj: RegistryModel) -> str:
        """Catalogue status of this row's chat id."""
        return model_status(obj.api_name)

    def get_credential_ref(self, obj: RegistryModel) -> str | None:
        """The provider's opaque ref. The secret is not on this row."""
        provider = getattr(obj, "provider", None)
        if provider is None:
            return None
        return provider.credential_ref


class ProviderAccountSchema(SQLAutoWithCamelCaseSchema):
    """One account and the models a personality may choose under it."""

    class Meta:
        model = Provider

    models = fields.Method("get_models", dump_only=True)

    def get_models(self, obj: Provider) -> list:
        listed = getattr(obj, "_listed_models", None)
        if listed is None:
            listed = [
                row
                for row in sorted(obj.models, key=lambda row: row.id)
                if is_listed_model(row)
            ]
        return provider_schema.dump(listed, many=True)


provider_schema = ProviderSchema()
providers_schema = ProviderSchema(many=True)
provider_accounts_schema = ProviderAccountSchema(many=True)
