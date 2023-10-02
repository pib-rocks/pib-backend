package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import org.threeten.bp.OffsetDateTime;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetVoiceAssistantChatMessagesMessages
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetVoiceAssistantChatMessagesMessages   {
  @JsonProperty("message")
  private String message = null;

  @JsonProperty("timestamp")
  private OffsetDateTime timestamp = null;

  @JsonProperty("isUser")
  private Boolean isUser = null;

  public GetVoiceAssistantChatMessagesMessages message(String message) {
    this.message = message;
    return this;
  }

  /**
   * Get message
   * @return message
   **/
  @Schema(example = "Hallo Thomas, erzähle mir etwas über Nürnberg.", description = "")
  
    public String getMessage() {
    return message;
  }

  public void setMessage(String message) {
    this.message = message;
  }

  public GetVoiceAssistantChatMessagesMessages timestamp(OffsetDateTime timestamp) {
    this.timestamp = timestamp;
    return this;
  }

  /**
   * Get timestamp
   * @return timestamp
   **/
  @Schema(example = "2021-01-30T08:30Z", description = "")
  
    @Valid
    public OffsetDateTime getTimestamp() {
    return timestamp;
  }

  public void setTimestamp(OffsetDateTime timestamp) {
    this.timestamp = timestamp;
  }

  public GetVoiceAssistantChatMessagesMessages isUser(Boolean isUser) {
    this.isUser = isUser;
    return this;
  }

  /**
   * Get isUser
   * @return isUser
   **/
  @Schema(example = "true", description = "")
  
    public Boolean isIsUser() {
    return isUser;
  }

  public void setIsUser(Boolean isUser) {
    this.isUser = isUser;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    GetVoiceAssistantChatMessagesMessages getVoiceAssistantChatMessagesMessages = (GetVoiceAssistantChatMessagesMessages) o;
    return Objects.equals(this.message, getVoiceAssistantChatMessagesMessages.message) &&
        Objects.equals(this.timestamp, getVoiceAssistantChatMessagesMessages.timestamp) &&
        Objects.equals(this.isUser, getVoiceAssistantChatMessagesMessages.isUser);
  }

  @Override
  public int hashCode() {
    return Objects.hash(message, timestamp, isUser);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetVoiceAssistantChatMessagesMessages {\n");
    
    sb.append("    message: ").append(toIndentedString(message)).append("\n");
    sb.append("    timestamp: ").append(toIndentedString(timestamp)).append("\n");
    sb.append("    isUser: ").append(toIndentedString(isUser)).append("\n");
    sb.append("}");
    return sb.toString();
  }

  /**
   * Convert the given object to string with each line indented by 4 spaces
   * (except the first line).
   */
  private String toIndentedString(java.lang.Object o) {
    if (o == null) {
      return "null";
    }
    return o.toString().replace("\n", "\n    ");
  }
}
