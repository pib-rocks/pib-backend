from model.chat_model import Chat
from app.app import ma
from marshmallow import fields


class ChatSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Chat
        exclude = ('id',)
    personalityId = fields.String()

chat_schema = ChatSchema()
chats_schema = ChatSchema(many=True)
upload_chat_schema = ChatSchema(only=['topic', 'personalityId'])