from model.chat_message_model import ChatMessage
from app.app import ma

class ChatMessageSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = ChatMessage
        exclude = ('id', 'chatId')

chat_message_schema = ChatMessageSchema()
chat_messages_schema = ChatMessageSchema(many=True)
chat_message_post_schema = ChatMessageSchema(only=('isUser', 'content'))