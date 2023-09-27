package io.swagger.api;

import io.swagger.model.CreateVoiceAssistantPersonality;
import io.swagger.model.DeleteVoiceAssistantPersonality;
import io.swagger.model.ErrorMessage;
import io.swagger.model.GetPersonalities;
import io.swagger.model.SuccessfulVoiceAssistantPersonalityCreationResponse;
import io.swagger.model.UpdateVoiceAssistantPersonality;
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

@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-09-27T09:57:56.878396287Z[GMT]")
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

    public ResponseEntity<GetPersonalities> voiceAssistantGet() {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<GetPersonalities>(objectMapper.readValue("{\n  \"voiceAssistantPersonalities\" : [ {\n    \"gender\" : \"Male/Female\",\n    \"name\" : \"Voice Assistant Personality Name\",\n    \"description\" : \"Personality Description\",\n    \"id\" : \"UUID\",\n    \"pauseThreshold\" : 0.8\n  }, {\n    \"gender\" : \"Male/Female\",\n    \"name\" : \"Voice Assistant Personality Name\",\n    \"description\" : \"Personality Description\",\n    \"id\" : \"UUID\",\n    \"pauseThreshold\" : 0.8\n  } ]\n}", GetPersonalities.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<GetPersonalities>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<GetPersonalities>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<DeleteVoiceAssistantPersonality> voiceAssistantIdDelete(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("id") String id) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<DeleteVoiceAssistantPersonality>(objectMapper.readValue("{\n  \"id\" : \"UUID\"\n}", DeleteVoiceAssistantPersonality.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<DeleteVoiceAssistantPersonality>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<DeleteVoiceAssistantPersonality>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<VoiceAssistantPersonality> voiceAssistantIdGet(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("id") String id) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<VoiceAssistantPersonality>(objectMapper.readValue("{\n  \"gender\" : \"Male/Female\",\n  \"name\" : \"Voice Assistant Personality Name\",\n  \"description\" : \"Personality Description\",\n  \"id\" : \"UUID\",\n  \"pauseThreshold\" : 0.8\n}", VoiceAssistantPersonality.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<VoiceAssistantPersonality>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<VoiceAssistantPersonality>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<UpdateVoiceAssistantPersonality> voiceAssistantIdPut(@Parameter(in = ParameterIn.PATH, description = "", required=true, schema=@Schema()) @PathVariable("id") String id,@Parameter(in = ParameterIn.DEFAULT, description = "Update an existing voice assistant personality<br>Name requires UTF-8 charset<br> Minimum length = X<br> Maximum length = X<br>Individual request body attributes may be nullable but all cannot be null", required=true, schema=@Schema()) @Valid @RequestBody UpdateVoiceAssistantPersonality body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<UpdateVoiceAssistantPersonality>(objectMapper.readValue("{\n  \"gender\" : \"Male/Female\",\n  \"name\" : \"Name of the voice assistant personality\",\n  \"description\" : \"Personality Description\",\n  \"id\" : \"UUID\",\n  \"pauseThreshold\" : 0.8\n}", UpdateVoiceAssistantPersonality.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<UpdateVoiceAssistantPersonality>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<UpdateVoiceAssistantPersonality>(HttpStatus.NOT_IMPLEMENTED);
    }

    public ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse> voiceAssistantPost(@Parameter(in = ParameterIn.DEFAULT, description = "Creates a new voice assistant personality<br>Name requires UTF-8 charset<br> Minimum length = X<br> Maximum length = X", required=true, schema=@Schema()) @Valid @RequestBody CreateVoiceAssistantPersonality body) {
        String accept = request.getHeader("Accept");
        if (accept != null && accept.contains("application/json")) {
            try {
                return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(objectMapper.readValue("{\n  \"id\" : \"UUID\"\n}", SuccessfulVoiceAssistantPersonalityCreationResponse.class), HttpStatus.NOT_IMPLEMENTED);
            } catch (IOException e) {
                log.error("Couldn't serialize response for content type application/json", e);
                return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(HttpStatus.INTERNAL_SERVER_ERROR);
            }
        }

        return new ResponseEntity<SuccessfulVoiceAssistantPersonalityCreationResponse>(HttpStatus.NOT_IMPLEMENTED);
    }

}
