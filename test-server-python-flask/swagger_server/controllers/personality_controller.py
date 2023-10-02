import connexion
import six

from swagger_server.models.create_voice_assistant_personality import CreateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_all_personalities import GetAllPersonalities  # noqa: E501
from swagger_server.models.successful_voice_assistant_personality_creation_response import SuccessfulVoiceAssistantPersonalityCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_personality import UpdateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.update_voice_assistant_personality_request import UpdateVoiceAssistantPersonalityRequest  # noqa: E501
from swagger_server.models.voice_assistant_personality import VoiceAssistantPersonality  # noqa: E501
from swagger_server import util


def voice_assistant_personality_get():  # noqa: E501
    """Get all voice-assistant personalities

     # noqa: E501


    :rtype: GetAllPersonalities
    """
    return 'do some magic!'


def voice_assistant_personality_personality_id_delete(personality_id):  # noqa: E501
    """Deletes voice assistant personality by id

     # noqa: E501

    :param personality_id: 
    :type personality_id: str

    :rtype: None
    """
    return 'do some magic!'


def voice_assistant_personality_personality_id_get(personality_id):  # noqa: E501
    """Get voice assistant by id

     # noqa: E501

    :param personality_id: 
    :type personality_id: str

    :rtype: VoiceAssistantPersonality
    """
    return 'do some magic!'


def voice_assistant_personality_personality_id_put(body, personality_id):  # noqa: E501
    """Create new voice assistant personality

     # noqa: E501

    :param body: Update an existing voice assistant personality&lt;br&gt;Description requires UTF-8 charset&lt;br&gt; Minimum length &#x3D; X&lt;br&gt; Maximum length &#x3D; X&lt;br&gt;Individual request body attributes may be nullable but all cannot be null
    :type body: dict | bytes
    :param personality_id: 
    :type personality_id: str

    :rtype: UpdateVoiceAssistantPersonality
    """
    if connexion.request.is_json:
        body = UpdateVoiceAssistantPersonalityRequest.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'


def voice_assistant_personality_post(body):  # noqa: E501
    """Create new voice assistant personality

     # noqa: E501

    :param body: Creates a new voice assistant personality&lt;br&gt;Description requires UTF-8 charset&lt;br&gt; Minimum length &#x3D; X&lt;br&gt; Maximum length &#x3D; X
    :type body: dict | bytes

    :rtype: SuccessfulVoiceAssistantPersonalityCreationResponse
    """
    if connexion.request.is_json:
        body = CreateVoiceAssistantPersonality.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'
