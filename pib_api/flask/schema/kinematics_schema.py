from marshmallow import EXCLUDE, Schema, fields, validate


class InverseRequestSchema(Schema):
    """Target for one arm, in metres in the REP-103 base frame."""

    class Meta:
        unknown = EXCLUDE

    side = fields.Str(required=True, validate=validate.OneOf(["left", "right"]))
    xyzM = fields.List(fields.Float(), required=True, validate=validate.Length(equal=3))
    rpyRad = fields.List(
        fields.Float(), load_default=None, validate=validate.Length(equal=3)
    )
    seedRad = fields.List(fields.Float(), load_default=None)


inverse_request_schema = InverseRequestSchema()
