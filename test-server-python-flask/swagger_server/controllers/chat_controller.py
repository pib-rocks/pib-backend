import connexion
import six

from swagger_server.models.add_voice_assistant_chat_message_request import AddVoiceAssistantChatMessageRequest  # noqa: E501
from swagger_server.models.add_voice_assistant_chat_message_response import AddVoiceAssistantChatMessageResponse  # noqa: E501
from swagger_server.models.create_voice_assistant_chat import CreateVoiceAssistantChat  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_all_chats import GetAllChats  # noqa: E501
from swagger_server.models.get_voice_assistant_chat import GetVoiceAssistantChat  # noqa: E501
from swagger_server.models.get_voice_assistant_chat_messages import GetVoiceAssistantChatMessages  # noqa: E501
from swagger_server.models.successful_voice_assistant_chat_creation_response import SuccessfulVoiceAssistantChatCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_chat import UpdateVoiceAssistantChat  # noqa: E501
from swagger_server.models.update_voice_assistant_chat_request import UpdateVoiceAssistantChatRequest  # noqa: E501
from swagger_server import util


def voice_assistant_chat_chat_id_delete(chat_id):  # noqa: E501
    """Deletes voice assistant personality by id

     # noqa: E501

    :param chat_id: 
    :type chat_id: str

    :rtype: None
    """
    return 'do some magic!'


def voice_assistant_chat_chat_id_get(chat_id):  # noqa: E501
    """Get voice assistant chat by id

     # noqa: E501

    :param chat_id: 
    :type chat_id: str

    :rtype: GetVoiceAssistantChat
    """
    return 'do some magic!'


def voice_assistant_chat_chat_id_messages_get(chat_id, timestamp=None, count=None):  # noqa: E501
    """Get voice assistant chat messages by chat id

     # noqa: E501

    :param chat_id: 
    :type chat_id: str
    :param timestamp: Null the first time. If specified, loads messages before the specified time stamp. Otherwise, loads the most recent messages based on count.
    :type timestamp: str
    :param count: Number of messages to be returned.
    :type count: float

    :rtype: GetVoiceAssistantChatMessages
    """
    return 'do some magic!'


def voice_assistant_chat_chat_id_messages_post(body, chat_id):  # noqa: E501
    """Get voice assistant chat by id

     # noqa: E501

    :param body: Adds prompt and the corresponding AI response to a chat based on chat id.
    :type body: dict | bytes
    :param chat_id: 
    :type chat_id: str

    :rtype: AddVoiceAssistantChatMessageResponse
    """
    if connexion.request.is_json:
        body = AddVoiceAssistantChatMessageRequest.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'


def voice_assistant_chat_chat_id_put(body, chat_id):  # noqa: E501
    """Create new voice assistant personality

     # noqa: E501

    :param body: Update an existing voice assistant chat&lt;br&gt;Individual request body attributes may be nullable but all cannot be null
    :type body: dict | bytes
    :param chat_id: 
    :type chat_id: str

    :rtype: UpdateVoiceAssistantChat
    """
    if connexion.request.is_json:
        body = UpdateVoiceAssistantChatRequest.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'


def voice_assistant_chat_get():  # noqa: E501
    """Get all voice-assistant chats

     # noqa: E501


    :rtype: GetAllChats
    """
    return 'do some magic!'


def voice_assistant_chat_post(body):  # noqa: E501
    """Create new voice assistant chat

     # noqa: E501

    :param body: Creates a new voice assistant chat
    :type body: dict | bytes

    :rtype: SuccessfulVoiceAssistantChatCreationResponse
    """
    if connexion.request.is_json:
        body = CreateVoiceAssistantChat.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'
