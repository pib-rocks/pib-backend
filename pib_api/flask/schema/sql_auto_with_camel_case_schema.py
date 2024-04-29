from app.app import ma


def camelcase(s: str) -> str:
    parts = iter(s.split("_"))
    return next(parts) + "".join(i.title() for i in parts)


class SQLAutoWithCamelCaseSchema(ma.SQLAlchemyAutoSchema):
    """Schema that uses camel-case for its external representation
    and snake-case for its internal representation.
    """

    @staticmethod
    def on_bind_field(field_name, field_obj):
        field_obj.data_key = camelcase(field_obj.data_key or field_name)
