from model.provider_model import Provider
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProviderSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Provider


provider_schema = ProviderSchema()
providers_schema = ProviderSchema(many=True)
