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
    # Budget: the drag loop asks for a fast answer, a one-off call can be patient.
    restarts = fields.Int(load_default=50, validate=validate.Range(min=0, max=200))
    maxIterations = fields.Int(load_default=200, validate=validate.Range(min=10, max=1000))


inverse_request_schema = InverseRequestSchema()
