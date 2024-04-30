from marshmallow import fields

from model.chat_model import Chat
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from schema.chat_message_schema import chat_messages_schema


class ChatSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Chat
        exclude = ('id',)

    personality_id = fields.String()
    messages = fields.Nested(chat_messages_schema)


chat_schema = ChatSchemaSQLAutoWith(exclude=('messages',))
chats_schema = ChatSchemaSQLAutoWith(many=True, exclude=('messages',))
upload_chat_schema = ChatSchemaSQLAutoWith(only=['topic', 'personality_id'])
chat_messages_only_schema = ChatSchemaSQLAutoWith(only=('messages',))
