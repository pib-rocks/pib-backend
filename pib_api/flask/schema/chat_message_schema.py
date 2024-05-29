from app.app import ma
from model.chat_message_model import ChatMessage


class ChatMessageSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = ChatMessage
        exclude = ("id", "chat_id")


chat_message_schema = ChatMessageSchema()
chat_messages_schema = ChatMessageSchema(many=True)
chat_message_post_schema = ChatMessageSchema(only=("is_user", "content"))
