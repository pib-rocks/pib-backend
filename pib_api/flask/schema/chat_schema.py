from marshmallow import fields
from model.chat_model import Chat
from pib_hermes_config.turn_taking import first_token_budget_ms
from schema.chat_message_schema import chat_messages_schema
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ChatSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Chat
        exclude = ("id",)

    personality_id = fields.String()
    messages = fields.Nested(chat_messages_schema)
    first_token_budget_ms = fields.Method("get_first_token_budget_ms", dump_only=True)

    def get_first_token_budget_ms(self, _obj: Chat) -> int:
        return first_token_budget_ms()


chat_schema = ChatSchemaSQLAutoWith(exclude=("messages",))
chats_schema = ChatSchemaSQLAutoWith(many=True, exclude=("messages",))
upload_chat_schema = ChatSchemaSQLAutoWith(only=["topic", "personality_id"])
chat_messages_only_schema = ChatSchemaSQLAutoWith(only=("messages",))
