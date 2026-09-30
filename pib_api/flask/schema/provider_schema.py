from marshmallow import fields

from model.provider_model import Provider
from provider_registry import model_status
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProviderSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Provider

    status = fields.Method("get_status", dump_only=True)

    def get_status(self, obj: Provider) -> str:
        """Catalogue status, so a retired row can be shown as gone."""
        return model_status(obj.api_name)


provider_schema = ProviderSchema()
providers_schema = ProviderSchema(many=True)
