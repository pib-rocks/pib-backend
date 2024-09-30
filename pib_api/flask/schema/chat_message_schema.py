from model.chat_message_model import ChatMessage
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from marshmallow import Schema, fields


class ChatMessageSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = ChatMessage
        exclude = ("id", "chat_id")


class ChatMessageDeltaSchema(Schema):
    delta = fields.Str(required=True)


chat_message_schema = ChatMessageSchemaSQLAutoWith()
chat_messages_schema = ChatMessageSchemaSQLAutoWith(many=True)
chat_message_post_schema = ChatMessageSchemaSQLAutoWith(only=("is_user", "content"))
chat_message_delta_schema = ChatMessageDeltaSchema()
