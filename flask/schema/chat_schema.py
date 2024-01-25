from model.chat_model import Chat
from app.app import ma
from schema.chat_message_schema import chat_messages_schema
from marshmallow import fields


class ChatSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Chat
        exclude = ('id',)
    personalityId = fields.String()
    messages = fields.Nested(chat_messages_schema)

chat_schema = ChatSchema(exclude=('messages',))
chats_schema = ChatSchema(many=True, exclude=('messages',))
upload_chat_schema = ChatSchema(only=['topic', 'personalityId'])
chat_messages_only_schema = ChatSchema(only=('messages',))