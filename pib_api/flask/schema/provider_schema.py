from marshmallow import fields

from model.provider_model import RegistryModel
from provider_registry import model_status
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProviderSchema(SQLAutoWithCamelCaseSchema):
    """A selectable model. providerId is the account the model belongs to."""

    class Meta:
        model = RegistryModel
        include_fk = True

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


provider_schema = ProviderSchema()
providers_schema = ProviderSchema(many=True)
