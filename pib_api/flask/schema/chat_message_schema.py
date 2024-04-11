from model.chat_message_model import ChatMessage
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class ChatMessageSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = ChatMessage
        exclude = ('id', 'chat_id')


chat_message_schema = ChatMessageSchemaSQLAutoWith()
chat_messages_schema = ChatMessageSchemaSQLAutoWith(many=True)
chat_message_post_schema = ChatMessageSchemaSQLAutoWith(only=('is_user', 'content'))
