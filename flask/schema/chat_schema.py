from model.chat_model import Chat
from app.app import ma

class ChatSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Chat
        exclude = ('id',)

chat_schema = ChatSchema()
chats_schema = ChatSchema(many=True)
upload_chat_schema = ChatSchema(exclude=('chatId',))