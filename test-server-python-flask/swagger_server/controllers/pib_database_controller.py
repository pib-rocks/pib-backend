import connexion
import six

from swagger_server.models.create_voice_assistant_personality import CreateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.delete_voice_assistant_personality import DeleteVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_personalities import GetPersonalities  # noqa: E501
from swagger_server.models.successful_voice_assistant_personality_creation_response import SuccessfulVoiceAssistantPersonalityCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_personality import UpdateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.voice_assistant_personality import VoiceAssistantPersonality  # noqa: E501
from swagger_server import util


def voice_assistant_get():  # noqa: E501
    """Get all voice-assistant personalities

     # noqa: E501


    :rtype: GetPersonalities
    """
    return 'do some magic!'


def voice_assistant_id_delete(id):  # noqa: E501
    """Deletes voice assistant personality by id

     # noqa: E501

    :param id: 
    :type id: str

    :rtype: DeleteVoiceAssistantPersonality
    """
    return 'do some magic!'


def voice_assistant_id_get(id):  # noqa: E501
    """Get voice assistant by id

     # noqa: E501

    :param id: 
    :type id: str

    :rtype: VoiceAssistantPersonality
    """
    return 'do some magic!'


def voice_assistant_id_put(body, id):  # noqa: E501
    """Create new voice assistant personality

     # noqa: E501

    :param body: Update an existing voice assistant personality&lt;br&gt;Name requires UTF-8 charset&lt;br&gt; Minimum length &#x3D; X&lt;br&gt; Maximum length &#x3D; X&lt;br&gt;Individual request body attributes may be nullable but all cannot be null
    :type body: dict | bytes
    :param id: 
    :type id: str

    :rtype: UpdateVoiceAssistantPersonality
    """
    if connexion.request.is_json:
        body = UpdateVoiceAssistantPersonality.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'


def voice_assistant_post(body):  # noqa: E501
    """Create new voice assistant personality

     # noqa: E501

    :param body: Creates a new voice assistant personality&lt;br&gt;Name requires UTF-8 charset&lt;br&gt; Minimum length &#x3D; X&lt;br&gt; Maximum length &#x3D; X
    :type body: dict | bytes

    :rtype: SuccessfulVoiceAssistantPersonalityCreationResponse
    """
    if connexion.request.is_json:
        body = CreateVoiceAssistantPersonality.from_dict(connexion.request.get_json())  # noqa: E501
    return 'do some magic!'
