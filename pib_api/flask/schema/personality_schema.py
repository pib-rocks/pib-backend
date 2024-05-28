from model.personality_model import Personality
from app.app import ma


class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality


personality_schema = PersonalitySchema(exclude=("id",))
upload_personality_schema = PersonalitySchema(exclude=("id", "personalityId"))
personalities_schema = PersonalitySchema(exclude=("id",), many=True)
