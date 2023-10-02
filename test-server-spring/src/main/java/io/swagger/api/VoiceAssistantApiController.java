package io.swagger.api;

import io.swagger.model.AddVoiceAssistantChatMessageRequest;
import io.swagger.model.AddVoiceAssistantChatMessageResponse;
import java.math.BigDecimal;
import io.swagger.model.CreateVoiceAssistantChat;
import io.swagger.model.CreateVoiceAssistantPersonality;
import io.swagger.model.ErrorMessage;
import io.swagger.model.GetAllChats;
import io.swagger.model.GetAllPersonalities;
import io.swagger.model.GetVoiceAssistantChat;
import io.swagger.model.GetVoiceAssistantChatMessages;
import io.swagger.model.SuccessfulVoiceAssistantChatCreationResponse;
import io.swagger.model.SuccessfulVoiceAssistantPersonalityCreationResponse;
import io.swagger.model.UpdateVoiceAssistantChat;
import io.swagger.model.UpdateVoiceAssistantChatRequest;
import io.swagger.model.UpdateVoiceAssistantPersonality;
import io.swagger.model.UpdateVoiceAssistantPersonalityRequest;
import io.swagger.model.VoiceAssistantPersonality;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.Parameter;
import io.swagger.v3.oas.annotations.enums.ParameterIn;
import io.swagger.v3.oas.annotations.responses.ApiResponses;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.media.ArraySchema;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.Schema;
import io.swagger.v3.oas.annotations.security.SecurityRequirement;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.bind.annotation.CookieValue;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestHeader;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RequestPart;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import javax.validation.Valid;
import javax.validation.constraints.*;
import javax.servlet.http.HttpServletRequest;
import java.io.IOException;
import java.util.List;
import java.util.Map;

@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")
@RestController
public class VoiceAssistantApiController implements VoiceAssistantApi {

    private static final Logger log = LoggerFactory.getLogger(VoiceAssistantApiController.class);

    private final ObjectMapper objectMapper;

    private final HttpServletRequest request;

    @org.springframework.beans.factory.annotation.Autowired
    public VoiceAssistantApiController(ObjectMapper objectMapper, HttpServletRequest request) {
        this.objectMapper = objectMapper;
        this.request = request;
    }

    public ResponseEntity<Void> voiceAssistantChatChatIdDelete(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("chat_id") String chatId) {
        String accept = request.getHeader("Accept");
        return new ResponseEntity<Void>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<GetVoiceAssistantChat> voiceAssistantChatChatIdGet(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("chat_id") String chatId) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<GetVoiceAssistantChat>(objectMapper.readValue("{\n  \"personality_id\" : \"UUID\",\n  \"personality_name\" : \"Thomas\",\n  \"title\" : \"Short Stories\"\n}", GetVoiceAssistantChat.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<GetVoiceAssistantChat>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<GetVoiceAssistantChat>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<GetVoiceAssistantChatMessages> voiceAssistantChatChatIdMessagesGet(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("chat_id") String chatId,@Parameter(in = ParameterIn.QUERY, description = "Null the first time. If specified, loads messages before the specified time stamp. Otherwise, loads the most recent messages based on count." ,schema=@Schema()) @Valid @RequestParam(value = "timestamp", required = false) String timestamp,@Parameter(in = ParameterIn.QUERY, description = "Number of messages to be returned." ,schema=@Schema( defaultValue="10")) @Valid @RequestParam(value = "count", required = false, defaultValue="10") BigDecimal count) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<GetVoiceAssistantChatMessages>(objectMapper.readValue("{\n  \"messages\" : [ {\n    \"message\" : \"Hallo Thomas, erzähle mir etwas über Nürnberg.\",\n    \"isUser\" : true,\n    \"timestamp\" : \"2021-01-30T08:30:00Z\"\n  }, {\n    \"message\" : \"Hallo Thomas, erzähle mir etwas über Nürnberg.\",\n    \"isUser\" : true,\n    \"timestamp\" : \"2021-01-30T08:30:00Z\"\n  } ]\n}", GetVoiceAssistantChatMessages.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<GetVoiceAssistantChatMessages>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<GetVoiceAssistantChatMessages>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<AddVoiceAssistantChatMessageResponse> voiceAssistantChatChatIdMessagesPost(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("chat_id") String chatId,@Parameter(in = ParameterIn.DEFAULT, description = "Adds prompt and the corresponding AI response to a chat based on chat id.", required=true, schema=@Schema()) @Valid @RequestBody AddVoiceAssistantChatMessageRequest body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<AddVoiceAssistantChatMessageResponse>(objectMapper.readValue("{\n  \"user_prompt\" : {\n    \"message\" : \"Hallo Thomas, erzähle mir etwas über Nürnberg.\",\n    \"isUser\" : true,\n    \"timestamp\" : \"2021-01-30T08:30:00Z\"\n  },\n  \"ai_response\" : {\n    \"message\" : \"Nürnberg ist eine Stadt in...\",\n    \"isUser\" : false,\n    \"timestamp\" : \"2021-01-30T08:31:00Z\"\n  }\n}", AddVoiceAssistantChatMessageResponse.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<AddVoiceAssistantChatMessageResponse>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<AddVoiceAssistantChatMessageResponse>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<UpdateVoiceAssistantChat> voiceAssistantChatChatIdPut(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("chat_id") String chatId,@Parameter(in = ParameterIn.DEFAULT, description = "Update an existing voice assistant chat<br>Individual request body attributes may be nullable but all cannot be null", required=true, schema=@Schema()) @Valid @RequestBody UpdateVoiceAssistantChatRequest body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<UpdateVoiceAssistantChat>(objectMapper.readValue("{\n  \"personality\" : \"Janina\",\n  \"title\" : \"Short Stories\"\n}", UpdateVoiceAssistantChat.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<UpdateVoiceAssistantChat>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<UpdateVoiceAssistantChat>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<GetAllChats> voiceAssistantChatGet() {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<GetAllChats>(objectMapper.readValue("{\n  \"voiceAssistantChats\" : [ {\n    \"personality_id\" : \"UUID\",\n    \"personality_name\" : \"Eva\",\n    \"title\" : \"Nuremberg\",\n    \"chat_id\" : \"UUID\"\n  }, {\n    \"personality_id\" : \"UUID\",\n    \"personality_name\" : \"Eva\",\n    \"title\" : \"Nuremberg\",\n    \"chat_id\" : \"UUID\"\n  } ]\n}", GetAllChats.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<GetAllChats>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<GetAllChats>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<SuccessfulVoiceAssistantChatCreationResponse> voiceAssistantChatPost(@Parameter(in = ParameterIn.DEFAULT, description = "Creates a new voice assistant chat", required=true, schema=@Schema()) @Valid @RequestBody CreateVoiceAssistantChat body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<SuccessfulVoiceAssistantChatCreationResponse>(objectMapper.readValue("{\n  \"personality_id\" : \"UUID\",\n  \"personality_name\" : \"Janina\",\n  \"title\" : \"Short Stories\",\n  \"chat_id\" : \"UUID\"\n}", SuccessfulVoiceAssistantChatCreationResponse.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<SuccessfulVoiceAssistantChatCreationResponse>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<SuccessfulVoiceAssistantChatCreationResponse>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<GetAllPersonalities> voiceAssistantPersonalityGet() {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<GetAllPersonalities>(objectMapper.readValue("{\n  \"voiceAssistantPersonalities\" : [ {\n    \"personality_id\" : \"UUID\",\n    \"gender\" : \"Male\",\n    \"name\" : \"Georg\",\n    \"description\" : \"Personality Description\",\n    \"pauseThreshold\" : 0.8\n  }, {\n    \"personality_id\" : \"UUID\",\n    \"gender\" : \"Male\",\n    \"name\" : \"Georg\",\n    \"description\" : \"Personality Description\",\n    \"pauseThreshold\" : 0.8\n  } ]\n}", GetAllPersonalities.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<GetAllPersonalities>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<GetAllPersonalities>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<Void> voiceAssistantPersonalityPersonalityIdDelete(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("personality_id") String personalityId) {
        String accept = request.getHeader("Accept");
        return new ResponseEntity<Void>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<VoiceAssistantPersonality> voiceAssistantPersonalityPersonalityIdGet(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("personality_id") String personalityId) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<VoiceAssistantPersonality>(objectMapper.readValue("{\n  \"personality_id\" : \"UUID\",\n  \"gender\" : \"Female\",\n  \"name\" : \"Janina\",\n  \"description\" : \"Personality Description\",\n  \"pauseThreshold\" : 0.8\n}", VoiceAssistantPersonality.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<VoiceAssistantPersonality>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<VoiceAssistantPersonality>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<UpdateVoiceAssistantPersonality> voiceAssistantPersonalityPersonalityIdPut(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("personality_id") String personalityId,@Parameter(in = ParameterIn.DEFAULT, description = "Update an existing voice assistant personality<br>Description requires UTF-8 charset<br> Minimum length = X<br> Maximum length = X<br>Individual request body attributes may be nullable but all cannot be null", required=true, schema=@Schema()) @Valid @RequestBody UpdateVoiceAssistantPersonalityRequest body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<UpdateVoiceAssistantPersonality>(objectMapper.readValue("{\n  \"personality_id\" : \"UUID\",\n  \"gender\" : \"Female\",\n  \"name\" : \"Eva\",\n  \"description\" : \"Personality Description\",\n  \"pauseThreshold\" : 0.8\n}", UpdateVoiceAssistantPersonality.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<UpdateVoiceAssistantPersonality>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<UpdateVoiceAssistantPersonality>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse> voiceAssistantPersonalityPost(@Parameter(in = ParameterIn.DEFAULT, description = "Creates a new voice assistant personality<br>Description requires UTF-8 charset<br> Minimum length = X<br> Maximum length = X", required=true, schema=@Schema()) @Valid @RequestBody CreateVoiceAssistantPersonality body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(objectMapper.readValue("{\n  \"personality_id\" : \"UUID\",\n  \"gender\" : \"Male\",\n  \"name\" : \"Thomas\",\n  \"description\" : \"Personality Description\",\n  \"pauseThreshold\" : 0.8\n}", SuccessfulVoiceAssistantPersonalityCreationResponse.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(HttpStatus.NOT_IMPLEMENTED);
    }

}
